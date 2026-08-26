#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped 
from geometry_msgs.msg import Twist

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)             # diff_drive_controller/cmd_vel_unstamped

        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)             # diff_drive_controller/cmd_vel_unstamped
        
        self.scale_vel_linear_x = 1.0  # m/s
        self.scale_vel_angular_z = 1.5  # rad/s
        
        self.get_logger().info("PS4 Teleop Node (TwistStamped) SUCCESS!")

    def joy_callback(self, msg: Joy):
        twist_stamped = Twist()

        # twist_stamped.header.stamp = self.get_clock().now().to_msg()
        # twist_stamped.header.frame_id = 'base_footprint'

        twist_stamped.linear.x = msg.axes[7] * self.scale_vel_linear_x
        twist_stamped.angular.z = msg.axes[6] * self.scale_vel_angular_z
        
        self.cmd_vel_pub.publish(twist_stamped)

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