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
            '23': {'status': 'available', 'pose': None, 'battery': 0, 'charge_time': 0},
            '10': {'status': 'available', 'pose': None, 'battery': 0, 'charge_time': 0},
            '68': {'status': 'available', 'pose': None, 'battery': 0, 'charge_time': 0},
        }

        # Subscription for robot status updates
        self.robot_status_subscriptions = {
            '23': self.create_subscription(String, '/robot_status_23', self.robot_status_callback_23, 10),
            '10': self.create_subscription(String, '/robot_status_10', self.robot_status_callback_10, 10),
            '68': self.create_subscription(String, '/robot_status_68', self.robot_status_callback_68, 10),
        }

        # Subscription for robot pose updates
        self.robot_pose_subscriptions = {
            '23': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_23', self.robot_pose_callback_23, 10),
            '10': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_10', self.robot_pose_callback_10, 10),
            '68': self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_68', self.robot_pose_callback_68, 10),
        }

        # Publisher for sending park numbers to robots
        self.send_park_num_publishers = {
            '23': self.create_publisher(String, '/send_park_num_23', 10),
            '10': self.create_publisher(String, '/send_park_num_10', 10),
            '68': self.create_publisher(String, '/send_park_num_68', 10),
        }

        # Publisher for robot battery status
        self.battery_status_publishers = {
            '23': self.create_publisher(String, '/robot_battery_23', 10),
            '10': self.create_publisher(String, '/robot_battery_10', 10),
            '68': self.create_publisher(String, '/robot_battery_68', 10),
        }

        # Subscription for receiving park numbers
        self.send_park_num_subscription = self.create_subscription(
            String,
            '/send_park_num',
            self.send_park_num_callback,
            10
        )

        # Subscription for task completion messages
        self.task_completion_subscription = {
            '23': self.create_subscription(String, '/send_park_num_23', self.task_completion_callback_23, 10),
            '10': self.create_subscription(String, '/send_park_num_10', self.task_completion_callback_10, 10),
            '68': self.create_subscription(String, '/send_park_num_68', self.task_completion_callback_68, 10),
        }

        # Subscription for task_completed1 messages
        self.task_completion_subscription1 = {
            '23': self.create_subscription(String, '/send_park_num_23', self.task_completion_callback1_23, 10),
            '10': self.create_subscription(String, '/send_park_num_10', self.task_completion_callback1_10, 10),
            '68': self.create_subscription(String, '/send_park_num_68', self.task_completion_callback1_68, 10),
        }

        # Subscription for robot battery status updates
        self.robot_battery_subscriptions = {
            '23': self.create_subscription(String, '/robot_battery_23', self.robot_battery_callback_23, 10),
            '10': self.create_subscription(String, '/robot_battery_10', self.robot_battery_callback_10, 10),
            '68': self.create_subscription(String, '/robot_battery_68', self.robot_battery_callback_68, 10),
        }

        # Timer to check charging status every second
        self.timer = self.create_timer(1.0, self.check_charging_status)

        # Load waypoints and routes
        self.waypoints = self.load_waypoints()
        self.routes = self.load_routes()

        # Initialize next_task
        self.next_task = None

    def load_waypoints(self):
        waypoints_file = '/home/ros2/pinkbot/src/pinklab_minibot_robot/forward_command/forward_command/waypoints.json'
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return []
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
        return data.get('waypoints', [])

    def load_routes(self):
        # Assuming the routes are in the same JSON file for simplicity
        waypoints_file = '/home/ros2/pinkbot/src/pinklab_minibot_robot/forward_command/forward_command/waypoints.json'
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return {}
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
        return data.get('routes', {})

    def robot_status_callback_23(self, msg):
        self.robots['23']['status'] = msg.data

    def robot_status_callback_10(self, msg):
        self.robots['10']['status'] = msg.data

    def robot_status_callback_68(self, msg):
        self.robots['68']['status'] = msg.data

    def robot_pose_callback_23(self, msg):
        self.robots['23']['pose'] = msg.pose.pose

    def robot_pose_callback_10(self, msg):
        self.robots['10']['pose'] = msg.pose.pose

    def robot_pose_callback_68(self, msg):
        self.robots['68']['pose'] = msg.pose.pose

    def robot_battery_callback_23(self, msg):
        try:
            # Extract the battery level from the received message
            battery_level = int(msg.data.lstrip('IR23'))  # Ensure correct substring
            self.robots['23']['battery'] = battery_level
        except ValueError:
            self.get_logger().warn(f'Invalid battery status received for robot 23: {msg.data}')

    def robot_battery_callback_10(self, msg):
        try:
            battery_level = int(msg.data.lstrip('IR10'))
            self.robots['10']['battery'] = battery_level
        except ValueError:
            self.get_logger().warn(f'Invalid battery status received for robot 10: {msg.data}')

    def robot_battery_callback_68(self, msg):
        try:
            battery_level = int(msg.data.lstrip('IR68'))
            self.robots['68']['battery'] = battery_level
        except ValueError:
            self.get_logger().warn(f'Invalid battery status received for robot 68: {msg.data}')


    def send_park_num_callback(self, msg):
        signal = msg.data

        if signal in ['RA2TG1', 'RA2TG2', 'RB2TG1', 'RB2TG2', 'RI1TE2']:
            self.handle_initial_signal(signal)
        elif signal in ['RA2TG1M']:
            self.second_initial_signal(signal)
        else:
            self.handle_other_signal(signal)

    def second_initial_signal(self, signal):
        available_robots = [r for r in self.robots.keys() if self.robots[r]['status'] == 'available' and self.robots[r]['battery'] >= 20]

        if len(available_robots) < 2:
            self.get_logger().warn('Not enough available robots with sufficient battery for second initial signal tasks.')
            return

        available_robots.sort(key=lambda r: self.get_distance_to_first_waypoint(self.robots[r]['pose']))

        closest_robot = available_robots[0]
        self.assign_task_to_robot(closest_robot, 'RE2TA2')
        self.deduct_battery(closest_robot)
        self.get_logger().info(f'Sent task RE2TA2 to robot {closest_robot}')

        next_closest_robot = available_robots[1]
        self.next_task = ('RA2TG1', next_closest_robot)
        self.get_logger().info(f'Set next task {self.next_task}')

    def task_completion_callback_23(self, msg):
        if msg.data == 'task_completed':
            if self.next_task:
                next_task, next_robot = self.next_task
                self.assign_task_to_robot(next_robot, next_task)
                self.deduct_battery(next_robot)
                self.get_logger().info(f'Sent task {next_task} to robot {next_robot}')
                self.next_task = None  # Clear next_task after assigning

    def task_completion_callback_10(self, msg):
        if msg.data == 'task_completed':
            if self.next_task:
                next_task, next_robot = self.next_task
                self.assign_task_to_robot(next_robot, next_task)
                self.deduct_battery(next_robot)
                self.get_logger().info(f'Sent task {next_task} to robot {next_robot}')
                self.next_task = None  # Clear next_task after assigning

    def task_completion_callback_68(self, msg):
        if msg.data == 'task_completed':
            if self.next_task:
                next_task, next_robot = self.next_task
                self.assign_task_to_robot(next_robot, next_task)
                self.deduct_battery(next_robot)
                self.get_logger().info(f'Sent task {next_task} to robot {next_robot}')
                self.next_task = None  # Clear next_task after assigning

    def task_completion_callback1_23(self, msg):
        if msg.data == 'task_completed1':
            # 모든 로봇 중에서 가장 가까운 로봇을 찾기
            target_waypoint = self.waypoints[0]  # 가장 가까운 거리 계산을 위한 타겟 웨이포인트
            closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_waypoint(self.robots[r]['pose'], target_waypoint))

            self.assign_task_to_robot(closest_robot, 'GE2TA22')
            self.deduct_battery(closest_robot)
            self.get_logger().info(f'Sent task GE2TA22 to robot {closest_robot}')
            self.next_task = None  # 작업을 할당한 후 next_task를 초기화

    # 아래 두 함수도 동일하게 수정합니다.
    def task_completion_callback1_10(self, msg):
        if msg.data == 'task_completed1':
            target_waypoint = self.waypoints[0]
            closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_waypoint(self.robots[r]['pose'], target_waypoint))

            self.assign_task_to_robot(closest_robot, 'GE2TA22')
            self.deduct_battery(closest_robot)
            self.get_logger().info(f'Sent task GE2TA22 to robot {closest_robot}')
            self.next_task = None

    def task_completion_callback1_68(self, msg):
        if msg.data == 'task_completed1':
            target_waypoint = self.waypoints[0]
            closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_waypoint(self.robots[r]['pose'], target_waypoint))

            self.assign_task_to_robot(closest_robot, 'GE2TA22')
            self.deduct_battery(closest_robot)
            self.get_logger().info(f'Sent task GE2TA22 to robot {closest_robot}')
            self.next_task = None

    def handle_initial_signal(self, signal):
        available_robots = [r for r in self.robots.keys() if self.robots[r]['status'] == 'available' and self.robots[r]['battery'] >= 20]

        if available_robots:
            closest_robot = min(available_robots, key=lambda r: self.get_distance_to_first_waypoint(self.robots[r]['pose']))
            self.assign_task_to_robot(closest_robot, signal)
            self.deduct_battery(closest_robot)
        else:
            self.get_logger().warn('No available robots with sufficient battery for initial signal tasks.')

    def handle_other_signal(self, signal):
        if signal in self.routes:
            first_waypoint_id = self.routes[signal][0]
            target_waypoint = self.get_waypoint_by_id(first_waypoint_id)
        else:
            target_waypoint = self.waypoints[0]

        closest_robot = min(self.robots.keys(), key=lambda r: self.get_distance_to_waypoint(self.robots[r]['pose'], target_waypoint))

        self.assign_task_to_robot(closest_robot, signal)

    def get_waypoint_by_id(self, waypoint_id):
        for waypoint in self.waypoints:
            if waypoint['id'] == waypoint_id:
                return waypoint
        return None

    def get_distance_to_first_waypoint(self, pose):
        if not pose or not self.waypoints:
            return float('inf')
        waypoint = self.waypoints[0]
        goal_x, goal_y = waypoint['x'], waypoint['y']
        current_x = pose.position.x
        current_y = pose.position.y
        return math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    def get_distance_to_waypoint(self, pose, waypoint):
        if not pose or not waypoint:
            return float('inf')
        goal_x, goal_y = waypoint['x'], waypoint['y']
        current_x = pose.position.x
        current_y = pose.position.y
        return math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    def assign_task_to_robot(self, robot_id, signal):
        msg = String()
        msg.data = signal
        self.send_park_num_publishers[robot_id].publish(msg)
        self.get_logger().info(f'Task "{signal}" assigned to robot {robot_id}')
        # Add log to confirm the task assignment
        self.get_logger().info(f'Task "{signal}" was sent to robot {robot_id} via topic {self.send_park_num_publishers[robot_id].topic}')

    def deduct_battery(self, robot_id):
        self.robots[robot_id]['battery'] = max(0, self.robots[robot_id]['battery'] - 10)
        self.publish_battery_status(robot_id)

    def check_charging_status(self):
        for robot_id, robot_info in self.robots.items():
            pose = robot_info['pose']
            if self.is_in_charging_area(pose):
                robot_info['charge_time'] += 1
                if robot_info['charge_time'] >= 5:
                    robot_info['battery'] = min(100, robot_info['battery'] + 5)
                    robot_info['charge_time'] = 0
                    self.publish_battery_status(robot_id)
            else:
                robot_info['charge_time'] = 0

            # Add logs to check robot status and battery
            self.get_logger().info(f'Robot {robot_id} status: {robot_info["status"]}, battery: {robot_info["battery"]}')


    def is_in_charging_area(self, pose):
        if pose is None:
            return False
        x, y = pose.position.x, pose.position.y
        return -0.2 <= x <= 0.2 and -0.45 <= y <= 0.45

    def publish_battery_status(self, robot_id):
        battery_level = self.robots[robot_id]['battery']
        msg = String()
        msg.data = f'IR{robot_id}{battery_level:03}'
        self.battery_status_publishers[robot_id].publish(msg)
        self.get_logger().info(f'Battery status for robot {robot_id}: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
