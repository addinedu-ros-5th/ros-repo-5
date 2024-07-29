import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import os
import json

class TaskManager(Node):
    def __init__(self):
        super().__init__('task')

        self.robots = {
            '23': {'status': 'available', 'pose': None},
            '10': {'status': 'available', 'pose': None},
            '59': {'status': 'available', 'pose': None},
        }

        # Subscription for robot status updates
        self.robot_status_subscriptions = {
            '23': self.create_subscription(String, '/robot_status_23', self.robot_status_callback_23, 10),
            '10': self.create_subscription(String, '/robot_status_10', self.robot_status_callback_10, 10),
            '59': self.create_subscription(String, '/robot_status_59', self.robot_status_callback_59, 10),
        }

        # Subscription for robot pose updates
        self.robot_pose_subscriptions = {
            '23': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_23', self.robot_pose_callback_23, 10),
            '10': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_10', self.robot_pose_callback_10, 10),
            '59': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_59', self.robot_pose_callback_59, 10),
        }

        # Publisher for sending park numbers to robots
        self.send_park_num_publishers = {
            '23': self.create_publisher(String, '/send_park_num_23', 10),
            '10': self.create_publisher(String, '/send_park_num_10', 10),
            '59': self.create_publisher(String, '/send_park_num_59', 10),
        }

        # Subscription for receiving park numbers
        self.send_park_num_subscription = self.create_subscription(
            String,
            '/send_park_num',
            self.send_park_num_callback,
            10
        )

        # Load waypoints
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        waypoints_file = '/home/ros2/pinkbot/src/pinklab_minibot_robot/forward_command/forward_command/waypoints.json'
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return []
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
        return data['waypoints']

    def robot_status_callback_23(self, msg):
        self.robots['23']['status'] = msg.data

    def robot_status_callback_10(self, msg):
        self.robots['10']['status'] = msg.data

    def robot_status_callback_59(self, msg):
        self.robots['59']['status'] = msg.data

    def robot_pose_callback_23(self, msg):
        self.robots['23']['pose'] = msg.pose.pose

    def robot_pose_callback_10(self, msg):
        self.robots['10']['pose'] = msg.pose.pose

    def robot_pose_callback_59(self, msg):
        self.robots['59']['pose'] = msg.pose.pose

    def send_park_num_callback(self, msg):
        signal = msg.data

        # Check if the signal is one of the initial signals
        if signal in ['RA2TG1', 'RA2TG2', 'RB2TG1', 'RB2TG2']:
            self.handle_initial_signal(signal)
        else:
            # Handle other signals
            self.handle_other_signal(signal)

    def handle_initial_signal(self, signal):
        # Find available robots
        available_robots = [r for r in self.robots.keys() if self.robots[r]['status'] == 'available']

        if available_robots:
            # If there are available robots, assign the task to the closest one
            closest_robot = min(available_robots, key=lambda r: self.get_distance_to_first_waypoint(self.robots[r]['pose']))
        else:
            # If no available robots, use the closest robot regardless of status
            closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_first_waypoint(self.robots[r]['pose']))

        self.assign_task_to_robot(closest_robot, signal)

    def handle_other_signal(self, signal):
        # Use the closest robot regardless of status
        closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_first_waypoint(self.robots[r]['pose']))

        self.assign_task_to_robot(closest_robot, signal)

    def assign_task_to_robot(self, robot_id, signal):
        if robot_id in self.robots:
            self.get_logger().info(f'Sending signal {signal} to robot {robot_id}')
            self.send_park_num_publishers[robot_id].publish(String(data=signal))
        else:
            self.get_logger().warning(f'Unknown robot id {robot_id}')

    def get_distance_to_first_waypoint(self, pose):
        if not pose or not self.waypoints:
            return float('inf')
        waypoint = self.waypoints[0]
        goal_x, goal_y = waypoint['x'], waypoint['y']
        current_x = pose.position.x
        current_y = pose.position.y
        return math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
