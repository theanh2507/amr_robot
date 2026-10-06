#!/usr/bin/env python3
import math
import time
from typing import List, Tuple, Optional

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    ReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


WAYPOINTS: List[Tuple[float, float, float]] = [
    (0.0, 0.0, 0.0),
    (9.0, 0.0, -90.0),
    (9.0, -2.5, -180.0),
    (0.0, -2.5, 90.0),
]

MAP_FRAME = 'map'
ROBOT_BASE_FRAME = 'base_footprint'

# So lan lap lai TOAN BO vong waypoint TRONG MOI LAN goi followPath. Vi
# followPath khong lap vo han duoc (phai tinh san 1 path huu han), chon so
# nay du lon de robot chay lien tuc mot thoi gian dai ma khong can goi lai.
LOOPS_PER_CALL = 200


def yaw_to_quaternion(yaw_deg: float) -> Tuple[float, float]:
    yaw_rad = math.radians(yaw_deg)
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quaternion_to_yaw_deg(qz: float, qw: float) -> float:
    yaw_rad = 2.0 * math.atan2(qz, qw)
    return math.degrees(yaw_rad)


class WaypointPatroller:
    """
    Chay TUAN TU theo DUNG 1 CHIEU co dinh (khong dao chieu), lap lai vong
    waypoint nhieu lan - nhung thay vi goi followPath() MOI CHU KY (gay loi
    SUCCEEDED gia vi diem dau/cuoi cua moi path trung nhau o waypoints[-1]),
    ta build SAN mot path dai noi LOOPS_PER_CALL vong lien tiep thanh MOT
    path DUY NHAT, roi chi goi followPath() MOT LAN cho ca chuoi do. Goal
    cuoi cung (diem cuoi path) chi xuat hien o CUOI CUNG, sau rat nhieu vong,
    nen khong con tinh huong "vua bat dau da trung goal" nhu truoc.
    """

    def __init__(self, waypoints: List[Tuple[float, float, float]], step_size: float = 0.05):
        self.waypoints = waypoints
        self.step_size = step_size

        self.nav = BasicNavigator()
        self.nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.nav, spin_thread=True)

        self.path_pub = self.nav.create_publisher(
            Path, '/plan',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

    # --------------------------------------------------------------
    def make_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def get_current_robot_pose_from_tf(self) -> Optional[Tuple[float, float, float]]:
        if not self.tf_buffer.can_transform(
                MAP_FRAME, ROBOT_BASE_FRAME, Time(), timeout=Duration(seconds=0.5)):
            self.nav.get_logger().warn(
                f'Chua tim frame {MAP_FRAME} trong cay TF, dang cho...',
                throttle_duration_sec=2.0)
            return None
        try:
            trans = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_BASE_FRAME, Time(), timeout=Duration(seconds=0.5))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            yaw_deg = quaternion_to_yaw_deg(qz, qw)
            return x, y, yaw_deg
        except Exception as e:
            self.nav.get_logger().warn(
                f'Loi khi lookup TF {MAP_FRAME} -> {ROBOT_BASE_FRAME}: {e}',
                throttle_duration_sec=2.0)
            return None

    def wait_for_robot_pose(self) -> Tuple[float, float, float]:
        pose = None
        while rclpy.ok() and pose is None:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            pose = self.get_current_robot_pose_from_tf()
            if pose is None:
                time.sleep(0.1)
        return pose

    def wait_for_amcl_pose(self):
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        received = {'ok': False}

        def amcl_callback(msg):
            if msg is not None:
                received['ok'] = True

        sub = self.nav.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', amcl_callback, amcl_pose_qos)
        while rclpy.ok() and not received['ok']:
            self.nav.get_logger().info('Dang cho amcl_pose')
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        self.nav.get_logger().info('Da nhan /amcl_pose! An toan de bat dau Nav2.')
        self.nav.destroy_subscription(sub)

    # --------------------------------------------------------------
    # Noi suy 1 doan thang giua 2 diem, THEM vao danh sach chung (khong tao
    # Path rieng) - dung lam khoi xay dung cho build_looped_path().
    # --------------------------------------------------------------
    def _append_segment(self, poses_out: list, p1: Tuple[float, float, float],
                         p2: Tuple[float, float, float]):
        x1, y1, yaw1 = p1
        x2, y2, yaw2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        num_steps = max(int(dist / self.step_size), 1)
        for s in range(num_steps):
            t = s / float(num_steps)
            curr_x = x1 + t * (x2 - x1)
            curr_y = y1 + t * (y2 - y1)
            diff_yaw = (yaw2 - yaw1 + 180.0) % 360.0 - 180.0
            curr_yaw = yaw1 + t * diff_yaw
            poses_out.append(self.make_pose(curr_x, curr_y, curr_yaw))

    # --------------------------------------------------------------
    # Build MOT path DUY NHAT: tu vi tri hien tai -> waypoints[0] -> ... ->
    # waypoints[-1] -> waypoints[0] -> ... -> waypoints[-1] (lap LOOPS_PER_CALL
    # lan), LUON DUNG 1 CHIEU co dinh (khong dao nguoc).
    # --------------------------------------------------------------
    def build_looped_path(self, start_pose: Tuple[float, float, float], loops: int) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = MAP_FRAME
        path_msg.header.stamp = self.nav.get_clock().now().to_msg()

        interpolated_poses: list = []

        # Doan dau: tu vi tri hien tai den waypoint dau tien
        self._append_segment(interpolated_poses, start_pose, self.waypoints[0])

        # Lap lai vong waypoint "loops" lan, noi tiep nhau khong dut quang
        for _ in range(loops):
            for i in range(len(self.waypoints) - 1):
                self._append_segment(interpolated_poses, self.waypoints[i], self.waypoints[i + 1])
            # Noi tu waypoint cuoi quay VE waypoint dau, de khep vong va tiep
            # tuc lap vong ke tiep lien tuc (tru lan lap CUOI CUNG, xu ly ben duoi)
            self._append_segment(interpolated_poses, self.waypoints[-1], self.waypoints[0])

        # Them diem CUOI CUNG that su (waypoint dau, vi vong lap tren luon
        # "chuan bi" cho 1 vong tiep theo) - xem lai logic: ta can path DUNG
        # tai waypoints[-1] cua vong lap CUOI, khong phai quay lai waypoints[0].
        # Sua lai bang cach bo doan quay-ve-dau o lan lap cuoi:
        last_x, last_y, last_yaw = self.waypoints[-1]
        interpolated_poses.append(self.make_pose(last_x, last_y, last_yaw))

        path_msg.poses = interpolated_poses
        return path_msg

    # --------------------------------------------------------------
    def run(self):
        self.wait_for_amcl_pose()
        self.nav.waitUntilNav2Active()

        try:
            while rclpy.ok():
                start_pose = self.wait_for_robot_pose()
                self.nav.get_logger().info(
                    f'Vi tri xuat phat (TF): ({start_pose[0]:.2f}, {start_pose[1]:.2f}, '
                    f'{start_pose[2]:.1f} deg) - bat dau {LOOPS_PER_CALL} vong lien tuc.')

                full_path = self.build_looped_path(start_pose, LOOPS_PER_CALL)
                self.nav.get_logger().info(f'Da noi suy path dai {len(full_path.poses)} diem.')

                self.path_pub.publish(full_path)
                self.nav.followPath(full_path)

                start_wait = time.time()
                while time.time() - start_wait < 0.3:
                    rclpy.spin_once(self.nav, timeout_sec=0.1)
                    time.sleep(0.02)

                while not self.nav.isTaskComplete():
                    rclpy.spin_once(self.nav)
                    feedback = self.nav.getFeedback()
                    if feedback:
                        print(f'   Con lai: {feedback.distance_to_goal:.2f} m', end='\r')
                    time.sleep(0.05)
                print()

                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.nav.get_logger().info(
                        f'Da hoan thanh {LOOPS_PER_CALL} vong - goi lai path moi de tiep tuc.')
                elif result == TaskResult.CANCELED:
                    self.nav.get_logger().warn('Bi huy - dung han chuong trinh.')
                    break
                else:
                    self.nav.get_logger().error('That bai giua chung - thu lai tu vi tri hien tai.')
                    time.sleep(1.0)

        except KeyboardInterrupt:
            self.nav.get_logger().info('Nhan Ctrl+C - dung robot va thoat.')
            self.nav.cancelTask()
        finally:
            self.nav.destroy_node()


def main():
    rclpy.init()
    patroller = WaypointPatroller(WAYPOINTS, step_size=0.05)
    try:
        patroller.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()