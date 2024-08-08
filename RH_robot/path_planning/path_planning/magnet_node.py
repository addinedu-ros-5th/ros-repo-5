import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
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
        self.publisher = self.create_publisher(String, 'send_park_num_()', 10) # () Robot ID
        self.distance_publisher = self.create_publisher(String, 'distance_measurement', 10)
        self.dock_state_publisher = self.create_publisher(Int32, 'dock_state', 10) # () Robot ID

        # GPIO 설정
        self.setup_gpio()
        self.get_logger().info('Node has been started.')

        # 상태 변수 초기화
        self.solenoid_on = False

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)  # BCM 모드 사용
        GPIO.setwarnings(False)  # 경고 메시지 비활성화

        self.TRIG = 23
        self.ECHO = 24
        self.MAGNET_PIN = 17  # BCM 핀 번호 17 (물리적 핀 번호 11)

        # 핀 설정
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.setup(self.MAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)

    def cleanup_gpio(self):
        GPIO.cleanup()

    def listener_callback(self, msg):
        park_num = msg.data
        self.get_logger().info(f'Received: {park_num}')

        # 거리 측정
        distance = self.measure_distance()
        distance_msg = String()
        distance_msg.data = f"Measured Distance: {distance} cm"
        self.distance_publisher.publish(distance_msg)
        self.get_logger().info(f'Published Distance: {distance_msg.data}')

        # 메시지 발행
        if park_num.startswith('H') and distance <= 5:
            park_num = 'G' + park_num[1:]
            self.get_logger().info(f'Publishing: {park_num}')
            new_msg = String()
            new_msg.data = park_num
            self.publisher.publish(new_msg)

        # 솔레노이드 상태 토글 및 dock_state 발행
        if distance <= 5:
            if not self.solenoid_on:
                # 솔레노이드 ON 상태로 변경
                GPIO.output(self.MAGNET_PIN, GPIO.HIGH)
                self.solenoid_on = True
                self.get_logger().info('Solenoid is now ON.')
                dock_state_msg = Int32()
                dock_state_msg.data = 1
                self.dock_state_publisher.publish(dock_state_msg)

            elif self.solenoid_on:
                # 솔레노이드 OFF 상태로 변경
                GPIO.output(self.MAGNET_PIN, GPIO.LOW)
                self.solenoid_on = False
                self.get_logger().info('Solenoid is now OFF.')
                dock_state_msg = Int32()
                dock_state_msg.data = 0
                self.dock_state_publisher.publish(dock_state_msg)
           
    def measure_distance(self):
        # 초음파 센서로 거리 측정
        GPIO.output(self.TRIG, False)
        time.sleep(2)

        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        pulse_start = pulse_end = None

        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()
        
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()

        if pulse_start and pulse_end:
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)
        else:
            distance = 0  # 오류 발생 시 기본값 설정
        
        self.get_logger().info(f'Measured Distance: {distance} cm')
        return distance

    def destroy_node(self):
        super().destroy_node()
        self.cleanup_gpio()

def main(args=None):
    rclpy.init(args=args)
    node = ParkNumberProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
