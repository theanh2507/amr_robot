#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRotator(Node):
    def __init__(self):
        super().__init__('scan_rotator')
        
        self.declare_parameter('rotation_deg', -90.0)
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan_rotate')
        self.declare_parameter('output_frame_id', 'Lidar_Link')

        rotation_deg = self.get_parameter('rotation_deg').value
        self.rotation_rad = math.radians(rotation_deg)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_profile_sensor_data)

        self.pub = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'scan_rotator: {input_topic} -> {output_topic}, '
            f'xoay {rotation_deg:.2f} do ({self.rotation_rad:.4f} rad)'
        )

    def scan_callback(self, msg: LaserScan):
        # Cach don gian nhat: chi can cong offset vao angle_min/angle_max.
        # Vi goc cua diem thu i luon duoc noi doc tinh la
        # angle_min + i * angle_increment, nen "dan nhan lai" goc bat dau
        # tuong duong hoan toan voi xoay toan bo du lieu, ma khong can
        # dung tay sap xep lai mang ranges[] (O(1) thay vi O(n)).
        out = LaserScan()

        out.header = msg.header
        out.header.frame_id = self.output_frame_id

        out.angle_min = msg.angle_min + self.rotation_rad
        out.angle_max = msg.angle_max + self.rotation_rad
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges          # giu nguyen, khong sap xep lai
        out.intensities = msg.intensities

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()