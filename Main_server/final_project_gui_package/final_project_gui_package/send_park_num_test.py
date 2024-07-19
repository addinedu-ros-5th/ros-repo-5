import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
import threading

class TestTest(Node):
    def __init__(self):
        super().__init__("test")
        self.test_subscriber = self.create_subscription(
            String,
            '/send_park_num',
            self.test_callback,
            10
        )
        self.test_publisher = self.create_publisher(String, '/send_park_num', 10)
        self.message_being_processed = False  # 메시지 처리 중 여부를 추적

    def test_callback(self, msg):
        if not self.message_being_processed:  # 메시지가 처리 중이 아니면
            self.message_being_processed = True
            temp = msg.data  # 문자열로 데이터를 가져옴
            self.get_logger().info(f'Received message: {temp}')
            
            # 3초 대기 후 발행
            threading.Timer(3.0, self.publish_message, args=(temp,)).start()

    def publish_message(self, message):
        # "R"을 추가한 새로운 메시지 생성
        new_msg = String()
        new_msg.data = message + "R"
        
        # 새로운 메시지 퍼블리시
        self.test_publisher.publish(new_msg)
        self.get_logger().info(f'Published message: {new_msg.data}')
        
        # 플래그를 초기화하여 다음 메시지를 받을 수 있게 함
        self.message_being_processed = False

def main(args=None):
    rp.init(args=args)
    test_tt = TestTest()
    rp.spin(test_tt)
    test_tt.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
