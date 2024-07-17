import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class From_ROS2_Signal_Publisher(Node):
    def __init__(self):
        super().__init__('from_ros2_signal_publisher')
        self.publisher = self.create_publisher(String, "/from_ros2_signal", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "ros2 signal Test"
        self.publisher.publish(msg)
        self.get_logger().info(f"publishing: '{msg.data}'")
        
        # 여기에 GUI에 해당하는 동작을 추가하면 됩니다.
        # 예를 들어, GUI의 라벨에 채우는 코드를 넣을 수 있습니다.

def main(args=None):
    rp.init(args=args)
    from_ros2_signal_publisher = From_ROS2_Signal_Publisher()
    
    rp.spin(from_ros2_signal_publisher)
    from_ros2_signal_publisher.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
