import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PlateTimestampSubscriber(Node):
    def __init__(self):
        super().__init__('plate_timestamp_subscriber')
        self.plate_subscription = self.create_subscription(
            String,
            'plate_topic',
            self.plate_listener_callback,
            10
        )
        self.in_time_subscription = self.create_subscription(
            String,
            'in_time_topic',
            self.in_time_listener_callback,
            10
        )
        self.out_time_subscription = self.create_subscription(
            String,
            'out_time_topic',
            self.out_time_listener_callback,
            10
        )
        self.plate_subscription  # prevent unused variable warning
        self.in_time_subscription  # prevent unused variable warning
        self.out_time_subscription  # prevent unused variable warning

        self.current_plate = None

    def plate_listener_callback(self, msg):
        self.current_plate = msg.data
        self.get_logger().info(f'Current plate set: {self.current_plate}')

    def in_time_listener_callback(self, msg):
        if self.current_plate:
            self.get_logger().info(f'CAR_NUM: {self.current_plate}, IN_TIME: {msg.data}')

    def out_time_listener_callback(self, msg):
        if self.current_plate:
            self.get_logger().info(f'CAR_NUM: {self.current_plate}, OUT_TIME: {msg.data}')
            self.current_plate = None  # Reset the current plate after OUT_TIME is published

def main(args=None):
    rclpy.init(args=args)
    node = PlateTimestampSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
