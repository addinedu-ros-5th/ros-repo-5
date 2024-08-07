import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import serial
from config import Config

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('robot_check')
        
        self.get_logger().info('Initializing ArduinoBridge...')
        
        self.subscription_send_park = self.create_subscription(
            String,
            '/send_park_num',
            self.send_park_callback,
            10)
        
        self.subscription_check_park = self.create_subscription(
            String,
            '/check_park_23',  # ROBOT ID
            self.check_park_callback,
            10)
        
        self.publisher_robot_check = self.create_publisher(String, '/robot_check', 10)
        self.publisher_dock_status = self.create_publisher(Int32, '/dock_status', 10)
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
        
        self.park_num_to_index = {
            'HA2': 1, 'HB2': 2, 'HE2': 3, 'HG1': 4,
            'HF1': 5, 'HO1': 6, 'HI1': 7
        }
        
        self.check_park_index = {
            'PA2', 'PB2', 'PE2', 'PG1',
            'PF1', 'PI1', 'PO1'
        }
        
        self.received_park_nums = {}
        self.dock_status = 0
        self.check_park_received = False  # Flag to ensure check_park message is received

        self.create_timer(0.1, self.read_from_arduino)
    
    def publish_dock_status(self):
        msg = Int32()
        msg.data = self.dock_status
        self.publisher_dock_status.publish(msg)
    
    def send_park_callback(self, msg):
        send_park_num = msg.data
        self.get_logger().info(f'Received park number: {send_park_num}')
        
        # Ensure the park number has the expected format
        if len(send_park_num) >= 3:
            park_num_prefix = send_park_num[:3]
            index = self.park_num_to_index.get(park_num_prefix, 0)  # Add default value of 0
        
            if index != 0:
                self.received_park_nums[park_num_prefix] = send_park_num
                try:
                    self.ser.write(f'{index}\n'.encode())
                    self.get_logger().info(f'Written index {index} for park number prefix {park_num_prefix} to serial port')
                except serial.SerialException as e:
                    self.get_logger().error(f'Failed to write to serial port: {e}')
            else:
                self.get_logger().error(f'Invalid park number prefix: {park_num_prefix}')
        else:
            self.get_logger().error(f'Received park number is too short: {send_park_num}')

    def check_park_callback(self, msg):
        check_park_num = msg.data
        self.check_park_received = True  # Set the flag when check_park message is received
        park_num_prefix = check_park_num[:3]
    
        if park_num_prefix in self.check_park_index:
            target_uid = Config.get_target_uid(check_park_num)  # Get target UID based on the entire park number
            self.get_logger().info(f"Target UID for {check_park_num}: {target_uid}")

            # Send solenoid toggle command only if dock_status is 1 and the target UID is matched
            if self.dock_status == 1 and check_park_num == target_uid['name']:
                try:
                    self.ser.write(b'toggle_solenoid\n')
                    self.get_logger().info('Sent solenoid toggle command')
                except serial.SerialException as e:
                    self.get_logger().error(f'Failed to write to serial port: {e}')

        
    def read_from_arduino(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("STATUS:"):
                    status_message = line[7:]  # Remove "STATUS:" prefix
                    if status_message != "RFID recognition ready. Give me your card." and \
                       status_message != "LEDState=true,solenoidState=true,rfidReady=true":
                        self.process_status_message(status_message)
                elif line.startswith("UID:"):
                    uid_message = f"Card UID: {line[4:]}"
                    self.get_logger().info(uid_message)
                    self.process_card_read()
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to read from serial port: {e}')
    
    def process_status_message(self, status_message):
        self.get_logger().info(f"Received status: {status_message}")
        if status_message == "Solenoid ON":
            self.dock_status = 1
            self.publish_dock_status()  # Publish dock status when it changes to 1
            
        elif status_message == "Solenoid OFF":
            self.dock_status = 0
            self.publish_dock_status()  # Publish dock status when it changes to 0

            # Publish a message to /robot_check with modified park number
            if self.received_park_nums:
                prefix, original = next(iter(self.received_park_nums.items()))
                new_message = 'G' + original[1:]
                msg = String()
                msg.data = new_message
                self.publisher_robot_check.publish(msg)
                self.get_logger().info(f'Published: {new_message}')
                self.received_park_nums.clear()

    def process_card_read(self):
        if self.received_park_nums:
            prefix, original = next(iter(self.received_park_nums.items()))
            new_message = 'G' + original[1:]
            msg = String()
            msg.data = new_message
            self.publisher_robot_check.publish(msg)
            self.get_logger().info(f'Published: {new_message}')
            self.received_park_nums.clear()
            self.dock_status = 0
            self.publish_dock_status()

def main(args=None):
    rclpy.init(args=args)
    robot_check = ArduinoBridge()
    rclpy.spin(robot_check)
    robot_check.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
