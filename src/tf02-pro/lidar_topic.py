import serial
import logging
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 

class IntPublisherNode(Node):
    def __init__(self):
        super().__init__('int_publisher_node')
        self.publisher_ = self.create_publisher(Int32, 'int_topic', 10)  
        self.timer = self.create_timer(0.5, self.publish_data)  
        self.msg_data = 0

    def publish_data(self):
        msg = Int32()
        msg.data = self.msg_data
        self.publisher_.publish(msg)

def read_serial_data(node):
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0) 

    while True:
        counter = ser.in_waiting
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3] * 256
                strength = bytes_serial[4] + bytes_serial[5] * 256
                temperature = bytes_serial[6] + bytes_serial[7] * 256
                temperature = (temperature / 8.0) - 256.0

                print('Distance: {0:2.2f} cm'.format(distance))

                if (distance < 10):
                    node.msg_data = 1
                else:
                    node.msg_data = 0 
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = IntPublisherNode()

    # Tạo một luồng mới để đọc dữ liệu từ cảm biến
    serial_thread = threading.Thread(target=read_serial_data, args=(node,))
    serial_thread.daemon = True  # Thiết lập luồng này là luồng nền
    serial_thread.start()  # Khởi động luồng đọc dữ liệu

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    # try:
    #     while True:
    #         # Gọi hàm publish_data từ luồng chính
    #         node.publish_data()
    #         time.sleep(0.5)  # Đảm bảo vòng lặp không làm tốn tài nguyên CPU

    # except KeyboardInterrupt:
    #     pass

    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()