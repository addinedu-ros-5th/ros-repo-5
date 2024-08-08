import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class ParkNumberProcessor(Node):

    def __init__(self):
        super().__init__('magnet_node')
        self.subscription = self.create_subscription(
            String,
            'send_park_num',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'send_park_num_68', 10)
        self.distance_publisher = self.create_publisher(String, 'distance_measurement', 10)  # 거리 측정 결과를 발행할 퍼블리셔

        # GPIO 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # 경고 메시지 비활성화
        self.TRIG = 23
        self.ECHO = 24
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        self.get_logger().info('Node has been started.')

    def listener_callback(self, msg):
        park_num = msg.data
        self.get_logger().info(f'Received: {park_num}')

        distance = self.measure_distance()
        distance_msg = String()
        distance_msg.data = f"Measured Distance: {distance} cm"
        self.distance_publisher.publish(distance_msg)  # 거리 측정 결과를 발행
        self.get_logger().info(f'Published Distance: {distance_msg.data}')
        
        if park_num.startswith('H') and distance <= 3:
            if park_num.startswith('H'):
                park_num = 'G' + park_num[1:]
            self.get_logger().info(f'Publishing: {park_num}')
            new_msg = String()
            new_msg.data = park_num
            self.publisher.publish(new_msg)

    def measure_distance(self):
        # 초음파 센서로 거리 측정
        GPIO.output(self.TRIG, False)
        time.sleep(2)

        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()
        
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        self.get_logger().info(f'Measured Distance: {distance} cm')
        return distance

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = ParkNumberProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
