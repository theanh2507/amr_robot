#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
from nav2_msgs.action import ComputeAndTrackRoute, ComputeRoute, FollowPath 

class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower_cli_bridge')
        self.route_client = ActionClient(self, ComputeRoute, '/compute_route')
        # self.route_client = ActionClient(self, ComputeAndTrackRoute, '/compute_and_track_route')
        self.track_client = ActionClient(self, FollowPath, '/follow_path')

    def run(self):
        # self.get_logger().info('Đang chờ các Action Server sẵn sàng...')
        self.route_client.wait_for_server()
        self.track_client.wait_for_server()
        # self.get_logger().info('Các Action Server đã sẵn sàng!')

        # Tạo goal cho ComputeRoute
        route_goal = ComputeRoute.Goal()
        route_goal.start_id = 6
        route_goal.goal_id = 2
        route_goal.use_poses = False  


        # send_goal_async tra ve ngay lap tuc 1 doi tuong future gan vao send_goal_future
        # send_goal_future dai dien cho 1 ket qua chua co o thoi diem hien tai nhung se co ket qua trong tuong lai khi action server phan hoi (chap nhan hoac tu choi)
        send_goal_future = self.route_client.send_goal_async(route_goal)        # gui route_goal den action server

        # add_done_callback: khi nao send_goal_future co ket qua (chap nhan hoac tu choi) thi goi ham goal_response_callback
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Yêu cầu tính toán tuyến đường bị từ chối!')
            return
        
        self.get_logger().info('Yêu cầu được chấp nhận! Đang lấy mảng tọa độ...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        # Lấy mảng đường đi từ trường route của ComputeRoute.Result
        extracted_path = result.path 
        
        # self.get_logger().info('=== ĐÃ NHẬN ĐƯỢC MẢNG TOẠ ĐỘ TỪ GRAPH ===')
        # print(extracted_path)
        # self.get_logger().info(f"Số lượng điểm tọa độ băm nhỏ: {len(extracted_path.poses)}")
        
        self.get_logger().info('Kích hoạt bộ bám đường (/follow_path) để robot di chuyển...')
        
        # Gửi sang bộ điều khiển local planner bám tuyến đường
        follow_goal = FollowPath.Goal()
        follow_goal.path = extracted_path
        follow_goal.controller_id = ''  # Sử dụng bộ Controller mặc định cấu hình trong Nav2
        
        self.track_client.send_goal_async(follow_goal)

def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
