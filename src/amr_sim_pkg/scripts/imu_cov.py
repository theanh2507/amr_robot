#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceFix(Node):
    def __init__(self):
        super().__init__('imu_covariance_fix')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_cov')

        # variance = stddev^2, dieu chinh theo do tin cay ban muon EKF danh cho IMU
        self.declare_parameter('orientation_stddev', 0.01)
        self.declare_parameter('angular_velocity_stddev', 0.001)
        self.declare_parameter('linear_acceleration_stddev', 0.05)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        ov = self.get_parameter('orientation_stddev').value ** 2
        av = self.get_parameter('angular_velocity_stddev').value ** 2
        la = self.get_parameter('linear_acceleration_stddev').value ** 2

        self.orientation_cov = [ov, 0.0, 0.0, 0.0, ov, 0.0, 0.0, 0.0, ov]
        self.angular_velocity_cov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        self.linear_acceleration_cov = [la, 0.0, 0.0, 0.0, la, 0.0, 0.0, 0.0, la]

        self.sub = self.create_subscription(Imu, in_topic, self.cb, 10)
        self.pub = self.create_publisher(Imu, out_topic, 10)

        self.get_logger().info(f"Relay covariance: {in_topic} -> {out_topic}")

    def cb(self, msg: Imu):
        msg.orientation_covariance = self.orientation_cov
        msg.angular_velocity_covariance = self.angular_velocity_cov
        msg.linear_acceleration_covariance = self.linear_acceleration_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()