import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CheckSubscriber(Node):
    def __init__(self):
        super().__init__('check_park_subscriber')
        self.plate_subscription = self.create_subscription(
            String,
            'check_park',  # Use the correct topic name
            self.plate_listener_callback,
            10
        )
        self.get_logger().info('Subscribed to check_park')

    def plate_listener_callback(self, msg):
        plate_number = msg.data
        self.get_logger().info(f'Received plate number: {plate_number}')
        # Add additional processing logic here

def main(args=None):
    rclpy.init(args=args)
    node = CheckSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
