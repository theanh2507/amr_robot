#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.scale_vel_linear_x = 0.7   # m/s
        self.scale_vel_angular_z = 1.0  # rad/s
        self.button_pose_map = {
            0: (0.0, 0.0),
            1: (10.0, -2.0),
            2: (10.0, 0.0),
            3: (0.0, 4.0)
        }
        
        self.get_logger().info("PS4 Teleop Node Cleaned SUCCESS!")

    def joy_callback(self, msg: Joy):
        is_l2_pressed = float(msg.axes[2] == -1.0)
        
        twist = Twist()
        twist.linear.x = msg.axes[7] * self.scale_vel_linear_x * is_l2_pressed
        twist.angular.z = msg.axes[6] * self.scale_vel_angular_z * is_l2_pressed
        
        if is_l2_pressed:
            self.cmd_vel_pub.publish(twist)

        pressed_buttons = [idx for idx, val in enumerate(msg.buttons[:4]) if val == 1]
        
        for btn_idx in pressed_buttons:
            if btn_idx in self.button_pose_map:
                x, y = self.button_pose_map[btn_idx]
                self._publish_pose(x, y)
                break 

    def _publish_pose(self, x: float, y: float):
        posestamp = PoseStamped()
        posestamp.header.frame_id = 'map'
        posestamp.header.stamp = self.get_clock().now().to_msg()

        posestamp.pose.position.x = x
        posestamp.pose.position.y = y
        posestamp.pose.position.z = 0.0

        posestamp.pose.orientation.w = 1.0

        self.pose_stamped_pub.publish(posestamp)


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