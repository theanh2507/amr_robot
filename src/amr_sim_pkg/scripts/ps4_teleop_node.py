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
        
        # Kiểm tra nút Deadman (Nút L1 trên PS4 tương ứng với buttons[4])
        # Nếu không nhấn L1, vận tốc trả về 0 để đảm bảo an toàn
        # if msg.buttons[4] == 1:
            # 1. Vận tốc thẳng (Tiến/Lùi): Lấy từ cần Analog Trái - trục đứng Y (axes[1])
        # twist.linear.x = msg.axes[1] * self.max_linear_x
        
        # # 2. Vận tốc ngang (Trái/Phải): Lấy từ cần Analog Trái - trục ngang X (axes[0])
        # # (Xóa hoặc comment dòng này nếu robot của bạn là xe 2 bánh vi sai thông thường)
        # twist.linear.y = msg.axes[0] * self.max_linear_y
        
        # # 3. Vận tốc góc (Xoay tại chỗ): Lấy từ cần Analog Phải - trục ngang X (axes[2])
        # twist.angular.z = msg.axes[2] * self.max_angular_z
        # else:
            # Không giữ L1 -> Dừng xe hoàn toàn
            # twist.linear.x = 0.0
            # twist.linear.y = 0.0
            # twist.angular.z = 0.0
            
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