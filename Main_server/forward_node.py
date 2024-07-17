import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ForwardNode(Node):
    def __init__(self):
        super().__init__('forward_node')
        self.subscription = self.create_subscription(
            String,
            '/from_gui_signal',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        if msg.data == '7777':
            twist = Twist()
            twist.angular.z = 1.0  # 회전 속도 설정
            self.publisher.publish(twist)
            self.get_logger().info('Robot is rotating for 2 seconds')
            self.create_timer(2.0, self.stop_rotation)

    def stop_rotation(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info('Rotation stopped')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
