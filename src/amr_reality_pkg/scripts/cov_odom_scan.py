#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovRepublisher(Node):
    def __init__(self):
        super().__init__('odom_cov_republisher')


        self.subscription = self.create_subscription(
            Odometry,
            '/odom_scan',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/odom_scan_cov',
            10
        )

        # Đường chéo chính: [X, Y, Z, Roll, Pitch, Yaw]
        self.custom_pose_covariance = [
            1.0, 0.0,   0.0,  0.0,  0.0,  0.0,      # X: Rất tin tưởng (0.002)
            0.0,   1.0, 0.0,  0.0,  0.0,  0.0,      # Y: Rất tin tưởng (0.002)
            0.0,   0.0,   1.0, 0.0,  0.0,  0.0,      # Z: Bỏ qua (2D)
            0.0,   0.0,   0.0,  1.0, 0.0,  0.0,      # Roll: Bỏ qua
            0.0,   0.0,   0.0,  0.0,  1.0, 0.0,      # Pitch: Bỏ qua
            0.0,   0.0,   0.0,  0.0,  0.0,  1.0   # Yaw: Nhiễu siêu lớn -> Phụ thuộc hoàn toàn IMU
        ]

        self.get_logger().info('Odom Covariance Republisher Node đã khởi chạy!')

    def odom_callback(self, msg: Odometry):
        msg.pose.covariance = self.custom_pose_covariance
        # Publish du lieu cho EKF
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCovRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()