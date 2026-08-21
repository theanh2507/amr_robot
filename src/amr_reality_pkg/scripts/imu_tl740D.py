#!/usr/bin/env python3

import serial
import time
import rclpy
import math
import tf_transformations
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class tl740D_imu(Node):
    def __init__(self):
        super().__init__("imu_sensor")

        self.declare_parameter("port", "/dev/ttyIMU")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "Imu_Link")
        self.declare_parameter("parent_frame", "base_link")
 
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.parent_frame = self.get_parameter("parent_frame").value

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,      
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        self.rx_buffer = bytearray()

        # use Best Effort policy, Volatile, KEEP_LAST voi depth = 5
        # UNITS: ACC la m/s^2, GYRO la rad/s, ANGLE la rad
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.imu_pub = self.create_publisher(Imu, "tl740D_data", qos_profile=qos)

        self.timer = self.create_timer(0.01, self.sensor_callback)

        # TF broadcaster: publish base_link -> Imu_Link de RViz xoay khoi hop theo IMU
        self.tf_broadcaster = TransformBroadcaster(self)


    def init_frequency_imu(self, rate_code=0x06):
        header = 0x68
        data_lenght = 0x05
        address = 0x00
        command_word = 0x0C
        data_domain = rate_code                                     # output frequency 100Hz

        # tinh checksum cua frame gui di
        checksum = (data_lenght + address + command_word + data_domain) % 256

        # tao goi tin gui di
        command_frame = bytearray([header, data_lenght, address, command_word, data_domain, checksum])

        self.ser.reset_input_buffer()
        self.ser.write(command_frame)
        self.ser.flush()                                            # blockk program until send data complete

        time.sleep(0.05)

        start_time = time.time()

        while(time.time() - start_time < 1.5):                      # time_out: 1.5s
            if self.ser.in_waiting >= 6:
                response = self.ser.read(1)                         # tra ve kieu du lieu bytes
                # print(response)

                if (response == b'\x68'):                           # b'\x68': data type la bytes, 0x68 datatype la int
                    rest = self.ser.read(5)

                    if(len(rest) < 5):
                        continue

                    combine_respone = response + rest

                    if(combine_respone[3] == 0x8C):
                        # tinh checksum cua frame nhan ve tu imu de so sanh voi gia tri checksum cua frame imu tra ve
                        ack_checksum = sum(combine_respone[1:5]) % 256

                        if(combine_respone[5] != ack_checksum):
                            self.get_logger().info("Wrong CheckSum")
                            return False

                        if(combine_respone[4] == 0x00):
                            self.get_logger().info("Config Success")
                            return True
                        else:
                            self.get_logger().info("Config Fail")
                            return False

                    else:
                        self.get_logger().info("No find response frame after config")
                        continue

        self.get_logger().info("No response")
        return False


    def parse_bcd_3bytes(self, b1, b2, b3, is_accel=False):
        # 1. Bit 0x10 quy định Dấu (-)
        sign = -1.0 if (b1 & 0x10) else 1.0
        
        # 2. Hàng trăm/đơn vị ở 4-bit thấp
        hundreds = (b1 & 0x0F)
        
        # 3. Đọc dạng Hex String để giữ nguyên giá trị BCD (VD: 0x23 -> "23")
        str_b2 = f"{b2:02x}"                # b2 = 0x04 -> f"{b2:02x}" = "04"
        str_b3 = f"{b3:02x}"

        try:
            if is_accel:
                # Gia tốc: X.XXX g (Ví dụ: 0x00 0x23 0x04 -> 2.304g)
                val_str = f"{hundreds}{str_b2[0]}.{str_b2[1]}{str_b3}"
            else:
                # Góc/Gyro: XXX.XX deg (Ví dụ: 0x10 0x50 0x23 -> -50.23 deg)
                val_str = f"{hundreds}{str_b2}.{str_b3}"
            
            return sign * float(val_str)
        except ValueError:
            return 0.0


    # tong do dai du lieu gom 32 bao gom ca byte SOF
    # Roll, Pitch, Yaw: degree
    # ACC_X, ACC_Y, ACC_Z: g
    # Gyro_X, Gyro_Y, Gyro_Z: degree/s  (van toc goc)

    def read_rion_format4_robust(self):
        if self.ser.in_waiting > 0:                             # kiem tra so luong byte dang co trong bo dem
            new_data = self.ser.read(self.ser.in_waiting)
            self.rx_buffer.extend(new_data)
            # print("RAW HEX:", new_data.hex())

        while len(self.rx_buffer) >= 31:
            if self.rx_buffer[0] != 0x68 or self.rx_buffer[1] != 0x1F:
                self.rx_buffer.pop(0) # Bỏ 1 byte rác
                continue

            frame = self.rx_buffer[:32]

            if frame[3] != 0x84:
                self.rx_buffer.pop(0)
                continue

            if len(frame) < 32:
                break
            
            # Checksum: Sum(Length -> Gyro_Z) % 256
            calc_checksum = sum(frame[1:31]) % 256
            
            if calc_checksum != frame[31]:
                print(f"[IMU Debug] Sai Checksum! Calc: {hex(calc_checksum)} != Recv: {hex(frame[31])}")
                self.rx_buffer.pop(0)
                continue

            # lay du lieu xong thi cat bo di 32 byte da doc, chi giu lai cac byte tu 32 den het
            self.rx_buffer = self.rx_buffer[32:]

            return {
                'roll':   self.parse_bcd_3bytes(frame[4],  frame[5],  frame[6]),
                'pitch':  self.parse_bcd_3bytes(frame[7],  frame[8],  frame[9]),
                'yaw':    self.parse_bcd_3bytes(frame[10], frame[11], frame[12]),
                'acc_x':  self.parse_bcd_3bytes(frame[13], frame[14], frame[15], is_accel=True),
                'acc_y':  self.parse_bcd_3bytes(frame[16], frame[17], frame[18], is_accel=True),
                'acc_z':  self.parse_bcd_3bytes(frame[19], frame[20], frame[21], is_accel=True),
                'gyro_x': self.parse_bcd_3bytes(frame[22], frame[23], frame[24]),
                'gyro_y': self.parse_bcd_3bytes(frame[25], frame[26], frame[27]),
                'gyro_z': self.parse_bcd_3bytes(frame[28], frame[29], frame[30])
            }

        return None

    def euler_to_quaternion(self, roll, pitch, yaw):
        # su dung chuoi quay 'sxyz' tuong duong voi phep quay zyx
        # quy uoc phep quay cua ros2 theo chaun REP103 la Intrinsic Z-Y-X (rzyx)         rzyx: phep quay dong, sxyz: phep quay tinh
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
        return q

    def sensor_callback(self):
        data = self.read_rion_format4_robust()

        if data is None:
            return

        # print(f"RPY: [{data['roll']:6.2f}, {data['pitch']:6.2f}, {data['yaw']:6.2f}] deg | "
        # f"ACC: [{data['acc_x']:6.3f}, {data['acc_y']:6.3f}, {data['acc_z']:6.3f}] g | "
        # f"GYRO: [{data['gyro_x']:6.2f}, {data['gyro_y']:6.2f}, {data['gyro_z']:6.2f}] deg/s")

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # --- Orientation: deg -> quaternion ---
        # dau vao ham euler_to_quaternion nhan don vi rad
        quaternion = self.euler_to_quaternion(math.radians(data['roll']), math.radians(data['pitch']), math.radians(data['yaw']))

        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        # Chua co so lieu covariance thuc te tu datasheet -> danh dau "unknown"
        # msg.angular_velocity_covariance[0] = -1.0
        msg.orientation_covariance[0] = 0.01   # roll variance
        msg.orientation_covariance[4] = 0.01   # pitch variance
        msg.orientation_covariance[8] = 0.01   # yaw variance -- nho hon odom de EKF tin IMU hon khi truot
 
        # --- Angular velocity: deg/s -> rad/s ---
        msg.angular_velocity.x = math.radians(data['gyro_x'])
        msg.angular_velocity.y = math.radians(data['gyro_y'])
        msg.angular_velocity.z = math.radians(data['gyro_z'])
        # msg.angular_velocity_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

 
        # --- Linear acceleration: g -> m/s^2 ---
        G = 9.80665
        msg.linear_acceleration.x = data['acc_x'] * G
        msg.linear_acceleration.y = data['acc_y'] * G
        msg.linear_acceleration.z = data['acc_z'] * G
        # msg.linear_acceleration_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
 
        self.imu_pub.publish(msg)

        # ---- 2. broadcast TF dong: world -> imu_link ----
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.03125
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
 
        # self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = tl740D_imu()
    node.init_frequency_imu(rate_code=0x05)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()