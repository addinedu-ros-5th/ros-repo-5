import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        self.get_logger().info('Initializing ArduinoBridge...')
        
        self.subscription = self.create_subscription(
            String,
            '/send_park_num',
            self.listener_callback,
            10)
        self.get_logger().info('Subscription created for topic "/send_park_num"')
        
        self.publisher = self.create_publisher(String, '/arduino_status', 10)
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
        
        self.park_num_to_index = {
            'HA2000': 1, 'HB2000': 2, 'HE2000': 3, 'HG1000': 4,
            'HF1000': 5, 'HO1000': 6, 'HI1000': 7
        }
        
        
        self.create_timer(0.1, self.read_from_arduino)
    
    def listener_callback(self, msg):
        send_park_num = msg.data
        self.get_logger().info(f'Received park number: {send_park_num}')
        
        index = self.park_num_to_index.get(send_park_num, 0)
        
        try:
            self.ser.write(f'{index}\n'.encode())
            self.get_logger().info(f'Written index {index} for park number {send_park_num} to serial port')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {e}')

    def read_from_arduino(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("STATUS:"):
                    msg = String()
                    msg.data = line[7:]  # Remove "STATUS:" prefix
                    self.publisher.publish(msg)
                elif line.startswith("UID:"):
                    msg = String()
                    msg.data = f"Card UID: {line[4:]}"
                    self.publisher.publish(msg)
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to read from serial port: {e}')

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = ArduinoBridge()
    rclpy.spin(arduino_bridge)
    arduino_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()