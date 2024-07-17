import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class From_GUI_Signal_Subscriber(Node):
    def __init__(self):
        super().__init__('from_gui_signal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/from_gui_signal',  # 여기에 발행된 토픽 이름을 입력해야 합니다.
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.data)
        # 여기에 GUI에 해당하는 동작을 추가하면 됩니다.
        # 예를 들어, GUI의 라벨에 채우는 코드를 넣을 수 있습니다.

def main(args=None):
    rp.init(args=args)
    from_gui_signal_subscriber = From_GUI_Signal_Subscriber()
    
    rp.spin(from_gui_signal_subscriber)
    from_gui_signal_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
