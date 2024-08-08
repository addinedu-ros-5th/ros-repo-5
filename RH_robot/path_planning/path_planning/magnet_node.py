import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParkNumberProcessor(Node):

    def __init__(self):
        super().__init__('magnet_node')
        self.subscription = self.create_subscription(
            String,
            'send_park_num',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'send_park_num_23', 10)
        self.get_logger().info('Node has been started.')

    def listener_callback(self, msg):
        park_num = msg.data
        self.get_logger().info(f'Received: {park_num}')

        if park_num.startswith('H'):
            park_num = 'G' + park_num[1:]
            self.get_logger().info(f'Publishing: {park_num}')
            new_msg = String()
            new_msg.data = park_num
            self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParkNumberProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
