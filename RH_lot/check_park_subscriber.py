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

class SignalSubscriber(Node):
    def __init__(self):
        super().__init__('signal_subscriber')
        self.signal_subscription = self.create_subscription(
            String,
            'lot_signal',
            self.signal_listener_callback,
            10
        )
        self.get_logger().info('Subscribed to lot_signal')

    def signal_listener_callback(self, msg):
        signal = msg.data
        self.get_logger().info(f'Received RFID signal: {signal}')
        # Add additional processing logic here

def main(args=None):
    rclpy.init(args=args)
    
    check_subscriber = CheckSubscriber()
    signal_subscriber = SignalSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(check_subscriber)
    executor.add_node(signal_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        check_subscriber.destroy_node()
        signal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()