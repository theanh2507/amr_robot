#!/usr/bin/env python3
import math
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# ============================================================
# CAU HINH DANH SACH DIEM MỤC TIÊU
# ============================================================
WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (5.0, 0.0, -180.0),
    (0.0, 0.0, 90.0),
    (0.0, 4.5, 0.0),
    (3.0, 4.5, -180.0),
    (0.0, 4.5, -90.0),
]

MAP_FRAME = 'map'
ROBOT_BASE_FRAME = 'base_footprint'


def yaw_to_quaternion(yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quaternion_to_yaw_deg(qz: float, qw: float) -> float:
    yaw_rad = 2.0 * math.atan2(qz, qw)
    return math.degrees(yaw_rad)


def make_pose(nav: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = MAP_FRAME
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw_deg)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def get_current_robot_pose_from_tf(tf_buffer: tf2_ros.Buffer, logger):
    # Kiểm tra xem liên kết giữa map và base_footprint đã sẵn sàng chưa
    if not tf_buffer.can_transform(MAP_FRAME, ROBOT_BASE_FRAME, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
        logger.warn(f'Chua tim me frame {MAP_FRAME} trong cây TF, dang cho...', throttle_duration_sec=2.0)
        return None

    try:
        trans = tf_buffer.lookup_transform(
            MAP_FRAME,
            ROBOT_BASE_FRAME,
            rclpy.time.Time(), # Lấy transform mới nhất
            timeout=rclpy.duration.Duration(seconds=0.5)
        )
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        yaw_deg = quaternion_to_yaw_deg(qz, qw)
        return x, y, yaw_deg
    except Exception as e:
        logger.warn(f'Loi khi lookup TF {MAP_FRAME} -> {ROBOT_BASE_FRAME}: {e}', throttle_duration_sec=2.0)
        return None


def interpolate_segment_from_current(nav: BasicNavigator, current_pose: tuple, target_wp: tuple, step_size: float = 0.05) -> Path:
    path_msg = Path()
    now_stamp = nav.get_clock().now().to_msg()
    path_msg.header.frame_id = MAP_FRAME
    path_msg.header.stamp = now_stamp

    x1, y1, yaw1 = current_pose
    x2, y2, yaw2 = target_wp

    dist = math.hypot(x2 - x1, y2 - y1)
    num_steps = max(int(dist / step_size), 1)

    interpolated_poses = []
    for s in range(num_steps):
        t = s / float(num_steps)
        curr_x = x1 + t * (x2 - x1)
        curr_y = y1 + t * (y2 - y1)

        diff_yaw = (yaw2 - yaw1 + 180.0) % 360.0 - 180.0
        curr_yaw = yaw1 + t * diff_yaw

        interpolated_poses.append(make_pose(nav, curr_x, curr_y, curr_yaw))

    interpolated_poses.append(make_pose(nav, x2, y2, yaw2))
    path_msg.poses = interpolated_poses
    return path_msg


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
    
    # Khoi tao TF Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, nav, spin_thread=True)   # spin_thread=True: spin TransformListener de cap nhat du lieu buffer tf

    amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

    amcl_pose_received = False

    def amcl_callback(msg):
        nonlocal amcl_pose_received 
        if(msg is not None):
            amcl_pose_received = True
        return

    sub = nav.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', amcl_callback, amcl_pose_qos)

    while rclpy.ok() and not amcl_pose_received:
        nav.get_logger().info("Dang cho amcl_pose")
        # time.sleep(0.1)
        rclpy.spin_once(nav, timeout_sec=0.1)
        # rclpy.spin(nav)

    nav.get_logger().info("Da nhan /amcl_pose! An toan de bat dau Nav2.")
    nav.destroy_subscription(sub)

    nav.waitUntilNav2Active()

    path_pub = nav.create_publisher(
    Path, '/plan',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE))

    cycle_count = 0

    try:
        while rclpy.ok():
            cycle_count += 1
            nav.get_logger().info(f'=== BAT DAU CHU KY #{cycle_count} ===')

            for i in range(len(WAYPOINTS)):
                target_wp = WAYPOINTS[i]

                # 1. DOC TOA DO THUC TE TU TF
                robot_pose = None
                while rclpy.ok() and robot_pose is None:
                    # Catch spin liên tục để cập nhật cây TF
                    rclpy.spin_once(nav)
                    robot_pose = get_current_robot_pose_from_tf(tf_buffer, nav.get_logger())
                    if robot_pose is None:
                        time.sleep(0.1)

                current_x, current_y, current_yaw = robot_pose

                # 2. KIEM TRA KHOANG CACH DEN WAYPOINT TARGET
                dist_to_target = math.hypot(target_wp[0] - current_x, target_wp[1] - current_y)
                if dist_to_target < 0.1:
                    nav.get_logger().info(
                        f'Robot da rat gan Waypoint {i+1} {target_wp[:2]} (cach {dist_to_target:.2f}m), chuyen tiep.'
                    )
                    continue

                nav.get_logger().info(
                    f'-> Toa do TF thuc te: ({current_x:.2f}, {current_y:.2f}, {current_yaw:.1f}deg) '
                    f'-> Noi suy den Waypoint {i+1}: {target_wp[:2]}'
                )

                # 3. NOI SUY DONG
                segment_path = interpolate_segment_from_current(
                    nav, (current_x, current_y, current_yaw), target_wp, step_size=0.05
                )

                path_pub.publish(segment_path)

                # 4. GUI PATH VA CHO ACTION ACCEPTED (Giai doan then chot)
                nav.followPath(segment_path)

                # CHO 0.3 GIÂY ĐỂ NAV2 ACTION SERVER NHẬN GOAL VÀ BẮT ĐẦU CHẠY
                # Tránh trường hợp check isTaskComplete ngay khi server chưa kịp ACCEPT Goal
                start_wait = time.time()
                while time.time() - start_wait < 0.3:
                    rclpy.spin_once(nav, timeout_sec=0.1)
                    time.sleep(0.02)

                # 5. VONG LAP CHO THUC THI CHUAN CUA ROS 2
                while not nav.isTaskComplete():
                    # BẮT BUỘC SPIN ĐỂ CẬP NHẬT ACTION CLIENT CALLBACK
                    rclpy.spin_once(nav)
                    
                    feedback = nav.getFeedback()
                    if feedback:
                        print(
                            f'   [Waypoint {i+1}] Khoang cach con lai: {feedback.distance_to_goal:.2f} m',
                            end='\r'
                        )
                    time.sleep(0.05)

                print() # Xuong dong


                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    nav.get_logger().info(f'-> DA DEN WAYPOINT {i+1}: {target_wp}')
                else:
                    nav.get_logger().error(f'Chang {i+1} THAT BAI hoac bi HUY!')
                    break

            nav.get_logger().info(f'=== HOAN THANH CHU KY #{cycle_count} ===\n')
            time.sleep(1.0)

    except KeyboardInterrupt:
        nav.get_logger().info('Dung robot va thoat.')
        nav.cancelTask()

    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# def main():
#     rclpy.init()

#     # 1. Khởi tạo BasicNavigator & Thêm tham số
#     nav = BasicNavigator()
#     nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

#     # 2. Sử dụng MultiThreadedExecutor để spin node ở background thread
#     # Giúp TF Buffer, Action feedback và Subscriptions luôn tự động cập nhật
#     executor = MultiThreadedExecutor()
#     executor.add_node(nav)
#     spin_thread = threading.Thread(target=executor.spin, daemon=True)
#     spin_thread.start()

#     # 3. Khởi tạo TF2 Buffer & Listener (Background spin sẽ tự cập nhật TF)
#     tf_buffer = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer, nav, spin_thread=True)

#     # 4. Đợi nhận pose đầu tiên từ AMCL
#     amcl_pose_received = False

#     def amcl_callback(msg):
#         nonlocal amcl_pose_received
#         amcl_pose_received = True

#     sub = nav.create_subscription(
#         PoseWithCovarianceStamped,
#         '/amcl_pose',
#         amcl_callback,
#         10
#     )

#     nav.get_logger().info("Dang cho tin hieu /amcl_pose...")
#     while rclpy.ok() and not amcl_pose_received:
#         time.sleep(0.1)

#     nav.get_logger().info("Da nhan /amcl_pose! An toan de bat dau Nav2.")
#     nav.destroy_subscription(sub)

#     # 5. Chờ Nav2 Active
#     nav.waitUntilNav2Active()

#     # 6. Khởi tạo Publisher cho Path
#     path_pub = nav.create_publisher(
#         Path,
#         '/plan',
#         QoSProfile(
#             depth=1,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=ReliabilityPolicy.RELIABLE
#         )
#     )

#     cycle_count = 0

#     try:
#         while rclpy.ok():
#             cycle_count += 1
#             nav.get_logger().info(f'=== BAT DAU CHU KY #{cycle_count} ===')

#             for i, target_wp in enumerate(WAYPOINTS):
#                 # 1. ĐỌC TỌA ĐỘ THỰC TẾ TỪ TF
#                 robot_pose = None
#                 while rclpy.ok() and robot_pose is None:
#                     robot_pose = get_current_robot_pose_from_tf(tf_buffer, nav.get_logger())
#                     if robot_pose is None:
#                         time.sleep(0.05)

#                 current_x, current_y, current_yaw = robot_pose

#                 # 2. KIỂM TRA KHOẢNG CÁCH ĐẾN WAYPOINT
#                 dist_to_target = math.hypot(target_wp[0] - current_x, target_wp[1] - current_y)
#                 if dist_to_target < 0.1:
#                     nav.get_logger().info(
#                         f'Robot da rat gan Waypoint {i+1} {target_wp[:2]} '
#                         f'(cach {dist_to_target:.2f}m), chuyen tiep.'
#                     )
#                     continue

#                 nav.get_logger().info(
#                     f'-> Pose TF: ({current_x:.2f}, {current_y:.2f}, {current_yaw:.1f}deg) '
#                     f'-> Noi suy toi Waypoint {i+1}: {target_wp[:2]}'
#                 )

#                 # 3. NỘI SỤY ĐƯỜNG ĐI DỘNG & PUBLISH PATH
#                 segment_path = interpolate_segment_from_current(
#                     nav, (current_x, current_y, current_yaw), target_wp, step_size=0.05
#                 )
#                 path_pub.publish(segment_path)

#                 # 4. GỬI GOAL CHO NAV2
#                 nav.followPath(segment_path)

#                 # 5. ĐỢI TASK HOÀN THÀNH (Executor ở background tự nhận feedback)
#                 while not nav.isTaskComplete():
#                     feedback = nav.getFeedback()
#                     if feedback:
#                         print(
#                             f'   [Waypoint {i+1}] Khoang cach con lai: {feedback.distance_to_goal:.2f} m',
#                             end='\r'
#                         )
#                     time.sleep(0.05)

#                 print()  # Xống dòng sau khi chạy xong waypoint

#                 # 6. KIỂM TRA KẾT QUẢ
#                 result = nav.getResult()
#                 if result == TaskResult.SUCCEEDED:
#                     nav.get_logger().info(f'-> DA DEN WAYPOINT {i+1}: {target_wp}')
#                 else:
#                     nav.get_logger().error(f'Chang {i+1} THAT BAI hoac bi HUY!')
#                     break

#             nav.get_logger().info(f'=== HOAN THANH CHU KY #{cycle_count} ===\n')
#             time.sleep(1.0)

#     except KeyboardInterrupt:
#         nav.get_logger().info('Nhan KeyboardInterrupt: Dang huystask va thoat...')
#         nav.cancelTask()

#     finally:
#         # Dọn dẹp tài nguyên
#         executor.shutdown()
#         nav.destroy_node()
#         rclpy.shutdown()