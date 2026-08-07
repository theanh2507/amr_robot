#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')
        
        # Subscriber để đọc dữ liệu từ tay cầm
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publisher để phát lệnh vận tốc cho robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Cấu hình giới hạn tốc độ tối đa cho robot (Có thể tùy chỉnh lại)
        self.max_linear_x = 0.5   # m/s (Tốc độ tiến lùi tối đa)
        self.max_linear_y = 0.5   # m/s (Tốc độ đi ngang tối đa - cho robot Omni)
        self.max_angular_z = 1.0  # rad/s (Tốc độ xoay tối đa)
        
        self.get_logger().info("PS4 Teleop Node đã khởi động thành công!")
        self.get_logger().info("GIỮ NÚT L1 để kích hoạt điều khiển robot.")

    def joy_callback(self, msg: Joy):
        twist = Twist()

        twist.linear.x = msg.axes[7] * self.max_linear_x
        twist.angular.z = msg.axes[6] * self.max_angular_z
            
        # Publish dữ liệu vận tốc ra topic /cmd_vel
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()