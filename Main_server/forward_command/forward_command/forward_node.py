import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import json
import os
import subprocess
from std_srvs.srv import Empty

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, pv, dt):
        error = setpoint - pv
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.subscription = self.create_subscription(
            String,
            '/send_park_num',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.amcl_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.send_park_num_publisher = self.create_publisher(String, '/send_park_num', 10)

        self.amcl_pose = Pose()  # Initialize amcl_pose

        self.waypoints_dict = self.load_waypoints()
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.tolerance = 0.2
        self.angle_tolerance = 0.2
        self.timer = self.create_timer(0.1, self.navigate_to_waypoints)
        self.reached_waypoint = False
        self.navigation_active = False

        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(1.0, 0.0, 0.1)

        self.last_time = self.get_clock().now()

        self.current_route = ""
        self.tg1_published = False  # 발행 플래그 추가
        self.post_action_performed = False

        self.srv = self.create_service(Empty, 'reset_node', self.reset_node_callback)

        # GOGO 신호와 대응하는 SA 신호 딕셔너리
        self.gogo_signal_to_park_num = {
            'GOGO': 'SA2TG1',
            'GOGO1': 'SA2TG2',
            'GOGO2': 'SB2TG1',
            'GOGO3': 'SB2TG2',
            'RG1TF1': 'SG1TF1',
            'RG1TO1': 'SG1TO1',
            'RF1TO1': 'SF1TO1'
        }

    def load_waypoints(self):
        waypoints_file = '/home/ros2/pinkbot/src/pinklab_minibot_robot/forward_command/forward_command/waypoints.json'
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return {}, {}
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
        waypoints = data['waypoints']
        routes = data['routes']
        return waypoints, routes

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        _, routes = self.waypoints_dict
        if msg.data in routes:
            self.get_logger().info(f'Starting waypoint navigation for route {msg.data}')
            waypoint_indices = routes[msg.data]
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = msg.data
            self.tg1_published = False  # Reset publish flag
            self.post_action_performed = False
        elif msg.data in self.gogo_signal_to_park_num:
            # GOGO 시그널 처리
            self.get_logger().info(f'Starting waypoint navigation for GOGO route {msg.data}')
            waypoint_indices = list(range(1, 5))  # 예시: GOGO 시그널에 대해 기본 waypoints 인덱스 설정
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = msg.data
            self.tg1_published = False  # Reset publish flag for GOGO signals
            self.post_action_performed = False
        else:
            self.get_logger().info(f'Invalid route or signal: {msg.data}')

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def get_current_pose(self):
        return self.amcl_pose

    def navigate_to_waypoints(self):
        if not self.navigation_active:
            return

        if self.current_waypoint_index >= len(self.current_waypoints):
            self.get_logger().info('All waypoints reached')
            self.navigation_active = False

            # 현재 경로에 따라 후속 작업 처리
            if self.current_route in ['RA2TG1', 'RB2TG1', 'RA2TG2', 'RB2TG2']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action()
            elif self.current_route in self.gogo_signal_to_park_num:
                if not self.tg1_published:
                    park_num_signal = self.gogo_signal_to_park_num[self.current_route]
                    self.get_logger().info(f'Publishing {park_num_signal} to /send_park_num')
                    msg = String()
                    msg.data = park_num_signal
                    self.send_park_num_publisher.publish(msg)
                    self.tg1_published = True  # 한 번만 발행되도록 설정
            return

        # self.get_logger().info('Navigating to waypoint')
        goal = self.current_waypoints[self.current_waypoint_index]
        goal_x, goal_y = goal['x'], goal['y']
        current_pose = self.get_current_pose()
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if distance < self.tolerance:
            if not self.reached_waypoint:
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached')
                self.reached_waypoint = True

            if self.reached_waypoint:
                self.get_logger().info(f'Orientation at waypoint {self.current_waypoint_index} achieved')
                self.current_waypoint_index += 1
                self.reached_waypoint = False
        else:
            angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
            current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
            angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

            cmd = Twist()
            if abs(angle_diff) > self.angle_tolerance:
                cmd.angular.z = self.angular_pid.compute(angle_diff, 0, dt)
            else:
                cmd.linear.x = self.linear_pid.compute(distance, 0, dt)
            self.publisher_.publish(cmd)

    def perform_post_navigation_action(self):
        # 제자리 회전 및 후진
        self.get_logger().info('Performing post-navigation rotation and backward movement')
        self.rotate_for_time(2.1)  # 90도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.move_backward_for_time(5)  # 5초간 후진

        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()

        # 노드 재시작
        self.restart_node()

    def rotate_for_time(self, angle):
        cmd = Twist()
        cmd.angular.z = 0.5  # 적절한 회전 속도로 설정
        duration = angle / 0.5
        self.publish_command_for_duration(cmd, duration)

    def move_backward_for_time(self, duration):
        cmd = Twist()
        cmd.linear.x = -0.1  # 적절한 후진 속도로 설정
        self.publish_command_for_duration(cmd, duration)

    def publish_command_for_duration(self, cmd, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
            # self.get_logger().info('stop0')
        self.publisher_.publish(Twist())  # 멈춤 명령 발행
        # self.get_logger().info('stop1')

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset_state(self):
        self.reached_waypoint = False
        self.navigation_active = False
        self.tg1_published = False
        self.post_action_performed = False
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.current_route = ""
        self.get_logger().info('State has been reset, ready for new commands.')

    def restart_node(self):
        self.get_logger().info('Restarting node...')
        subprocess.Popen(['ros2', 'run', 'forward_command', 'forward_node'])
        self.get_logger().info('Node restart command issued')

    def reset_node_callback(self, request, response):
        self.get_logger().info('Received reset_node request. Restarting node...')
        self.reset_state()
        subprocess.Popen(['ros2', 'run', 'forward_command', 'forward_node'])
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
