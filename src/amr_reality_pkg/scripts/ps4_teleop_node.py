#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped


class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.cmd_vel_joy_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.cmd_vel_override_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.scale_vel_linear_x = 0.3
        self.scale_vel_angular_z = 0.4

        self.button_pose_map = {
            0: (0.0, 0.0),
            1: (10.0, -2.0),
            2: (10.0, 0.0),
            3: (0.0, 4.0)
        }

        # Index nut - kiem tra dung bang: ros2 topic echo /joy
        self.deadman_axis_index = 2       # L2 trigger (axes[2] == -1.0 khi giu het co)
        self.override_button_index = 5    # vi du R1 - nut giu de "vuot rao" khi lui

        # Chong spam publish goal khi giu nut lau
        self.prev_pressed_buttons = set()

        self.get_logger().info("PS4 Teleop Node SUCCESS!")

    def joy_callback(self, msg: Joy):
        is_deadman = msg.axes[self.deadman_axis_index] <= -0.9   # threshold, tranh sai so analog (nhan L2)
        is_override = (len(msg.buttons) > self.override_button_index and
                       msg.buttons[self.override_button_index] == 1)

        if is_deadman:
            twist = Twist()
            twist.linear.x = msg.axes[7] * self.scale_vel_linear_x
            twist.angular.z = msg.axes[6] * self.scale_vel_angular_z

            # Chi bypass collision_monitor khi CA 2 dieu kien: giu nut override RIENG + dang lui
            if is_override and twist.linear.x < 0.0:
                self.cmd_vel_override_pub.publish(twist)
                self.get_logger().warn(
                    'BYPASS collision_monitor - lui khan cap theo lenh nguoi van hanh',
                    throttle_duration_sec=1.0)
            else:
                # Duong binh thuong - di qua twist_mux -> collision_monitor
                self.cmd_vel_joy_pub.publish(twist)

        self._handle_goal_buttons(msg)

    def _handle_goal_buttons(self, msg: Joy):
        pressed_now = {idx for idx, val in enumerate(msg.buttons[:4]) if val == 1}
        # Chi xu ly nut VUA duoc bam xuong (rising edge), khong lap lai khi dang giu
        newly_pressed = pressed_now - self.prev_pressed_buttons
        self.prev_pressed_buttons = pressed_now

        for btn_idx in newly_pressed:
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