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
        super().__init__('path_planning_node')

        # Subscription to receive waypoint signals
        self.subscription = self.create_subscription(
            String,
            '/send_park_num_23',
            self.listener_callback,
            10)

        # Publisher for cmd_vel to control the robot
        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        # Subscription to AMCL pose for robot localization
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10)

        # Publisher to notify the server that the route is completed
        self.send_park_num_publisher = self.create_publisher(String, '/send_park_num', 10)

        # Publisher for robot status
        self.robot_status_publisher = self.create_publisher(String, '/robot_status', 10)

        self.amcl_pose = Pose()  # Initialize amcl_pose

        # Load waypoints from JSON file
        self.waypoints_dict = self.load_waypoints()
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.tolerance = 0.2
        self.angle_tolerance = 0.2
        self.timer = self.create_timer(0.1, self.navigate_to_waypoints)
        self.reached_waypoint = False
        self.navigation_active = False

        # PID controllers for linear and angular movements
        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(1.0, 0.0, 0.1)
        self.last_time = self.get_clock().now()
        self.current_route = ""
        self.tg1_published = False  # Flag to ensure signal is published only once
        self.post_action_performed = False
        self.srv = self.create_service(Empty, 'reset_node', self.reset_node_callback)

        # GOGO 신호와 대응하는 SA 신호 딕셔너리
        self.gogo_signal_to_park_num = {
            'GA2TG1': 'SA2TG1',
            'GA2TG2': 'SA2TG2',
            'GB2TG1': 'SB2TG1',
            'GB2TG2': 'SB2TG2',
            'RG1TF1': 'SG1TF1',
            'RG1TO1': 'SG1TO1',
            'RF1TO1': 'SF1TO1'
        }

        # 작업 중인 상태로 설정할 신호 목록
        self.busy_signals = ['RA2TG1', 'RA2TG2', 'RB2TG1', 'RB2TG2']

        # 작업 가능 상태로 설정할 신호 목록
        self.available_signals = [
            'GG1TF1', 'GG1TF2', 'GG2TF1', 'GG2TF2',
            'GG1TO1', 'GG2TO1',
            'GF1TO1', 'GF2TO1',
            'GA2TR1', 'GA2TR2', 'GA2TR3',
            'GB2TR1', 'GB2TR2', 'GB2TR3'
        ]

    def load_waypoints(self):
        waypoints_file = '/home/john/pinkbot/src/pinklab_minibot_robot/path_planning/path_planning/waypoints.json'
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
        data = msg.data

        # Handle robot status changes based on signals
        if data in self.busy_signals:
            self.publish_robot_status('busy')
        elif data in self.available_signals:
            self.publish_robot_status('available')

        # Check if the signal is related to navigation
        if data.startswith('23'):
            data = data[2:]
        
        _, routes = self.waypoints_dict
        if data in routes:
            self.get_logger().info(f'Starting waypoint navigation for route {data}')
            waypoint_indices = routes[data]
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = data
            self.tg1_published = False  # Reset publish flag
            self.post_action_performed = False
        elif data in self.gogo_signal_to_park_num:
            self.get_logger().info(f'Starting waypoint navigation for GOGO route {data}')
            waypoint_indices = list(range(1, 5))  # 예시: GOGO 시그널에 대해 기본 waypoints 인덱스 설정
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = data
            self.tg1_published = False  # Reset publish flag for GOGO signals
            self.post_action_performed = False
        else:
            self.get_logger().info(f'Invalid route or signal: {data}')

    def publish_robot_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.robot_status_publisher.publish(status_msg)
        self.get_logger().info(f'Published robot status: {status}')

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
            if self.current_route in ['RA2TG1', 'RA2TG2']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_left()
            elif self.current_route in ['RB2TG1', 'RB2TG2']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_right()
            elif self.current_route in ['RG1TF1', 'RG2TF1']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_left()
            elif self.current_route in ['RG1TF2', 'RG2TF2']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_left2()
            elif self.current_route in ['RI1TA2','RI1TB2', 'RI1TE2']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_right_180()
            elif self.current_route in ['RF1TO1', 'RF2TO1']:
                if not self.post_action_performed:
                    self.perform_post_navigastion_action_right2()
            elif self.current_route in ['GF1TO1', 'GF2TO1']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_rotation_right_90()
            elif self.current_route in ['RI1TE2', '매복 출차']:
                if not self.post_action_performed:
                    self.perform_post_navigation_action_rotation_second_park()
            elif self.current_route in self.gogo_signal_to_park_num:
                if not self.tg1_published:
                    park_num_signal = self.gogo_signal_to_park_num[self.current_route]
                    self.get_logger().info(f'Publishing {park_num_signal} to /send_park_num')
                    msg = String()
                    msg.data = park_num_signal
                    self.send_park_num_publisher.publish(msg)
                    self.tg1_published = True  # 한 번만 발행되도록 설정

            return

        # Waypoint로 내비게이션
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

    def perform_post_navigation_action_left(self):
        # 제자리 회전 및 후진
        self.get_logger().info('Performing post-navigation rotation and backward movement')
        self.rotate_for_time_left(2.1)  # 90도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.move_backward_for_time(5)  # 5초간 후진
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        # 노드 재시작
        self.restart_node()

    def perform_post_navigation_action_right(self):
        self.get_logger().info('Performing post-navigation rotation and backward movement')
        self.rotate_for_time_right(2.1)  # 90도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.move_backward_for_time(5)  # 5초간 후진
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        # 노드 재시작
        self.restart_node() 

    def perform_post_navigation_action_right_180(self):
        self.get_logger().info('Performing post-navigation rotation and backward movement')
        self.rotate_for_time_right(4.2)  # 90도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        # 노드 재시작
        self.restart_node() 

    # 제자리 90도 왼쪽 회전
    def perform_post_navigation_action_left2(self):
        self.get_logger().info('Performing post-navigation rotation')
        self.rotate_for_time_left(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    def perform_post_navigastion_action_right2(self):
        self.get_logger().info('Performing post-navigation rotation')
        self.rotate_for_time_right(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()
    
    # 우회전 
    def perform_post_navigation_action_rotation_right_90(self):
        self.get_logger().info('Performing post-navigation rotation right')
        self.rotate_for_right(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.get_logger.info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()
    
    # 이중 주차 함수
    def perform_post_navigation_action_rotation_second_park(self):
        self.get_logger().info('Performing post-navigation rotation second park')
        self.rotate_for_curve(1)
        self.rotate_for_curve2(1)
        rclpy.spin_once(self, timout_sec=2)
        self.rotate_for_curve3(1)
        self.rotate_for_curve4(1)
        self.get_logger().info('Post-navigation action completed')
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    def rotate_for_time_left(self, angle):
        cmd = Twist()
        cmd.angular.z = 0.5  # 적절한 회전 속도로 설정
        duration = angle / 0.5
        self.publish_command_for_duration(cmd, duration)
    
    def rotate_for_time_right(self, angle):
        cmd = Twist()
        cmd.angular.z = -0.5
        duration = angle / 0.5
        self.publish_command_for_duration(cmd, duration)

    # 우회전 함수
    def rotate_for_right(self, duration):
        cmd = Twist()
        cmd.linear.x = 0.1  # 직진 속도 설정 (양수는 전진, 음수는 후진)
        cmd.angular.z = -0.2  # 각속도 설정 (음수는 우회전, 양수는 좌회전)
        self.publish_command_for_duration(cmd, duration)
            
    # 이중 주차 곡선 함수 (주차)
    def rotate_for_curve(self, duration):
        cmd = Twist()
        cmd.linear.x = -0.1
        cmd.angular.z = 0.2
        self.publish_command_for_duration(cmd, duration) 
    def rotate_for_curve2(self, duration):
        cmd = Twist()
        cmd.linear.x = -0.1
        cmd.angular.z = -0.2
        self.publish_command_for_duration(cmd, duration)

    # 이중 주차 곡선 함수 (출차)
    def rotate_for_curve3(self, duration):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -0.2
        self.publish_command_for_duration(cmd, duration)
    def rotate_for_curve4(self, duration):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = 0.2
        self.publish_command_for_duration(cmd, duration)


    # def rotate_for_time_right_180(self, angle):
    #     cmd = Twist()
    #     cmd.angular.z = -0.5
    #     duration = angle / 0.5
    #     self.publish_command_for_duration_180(cmd, duration)

    def move_backward_for_time(self, time):
        cmd = Twist()
        cmd.linear.x = -0.1  # 적절한 후진 속도로 설정
        self.publish_command_for_duration(cmd, time)

    def publish_command_for_duration(self, cmd, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.publisher_.publish(Twist())  # 멈춤 명령 발행

    # def publish_command_for_duration_180(self, cmd, duration):
    #     start_time = self.get_clock().now()
    #     while (self.get_clock().now() - start_time).nanoseconds / 2e9 < duration:
    #         self.publisher_.publish(cmd)
    #         rclpy.spin_once(self, timeout_sec=0.1)
    #     self.publisher_.publish(Twist())

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
        subprocess.Popen(['ros2', 'run', 'path_planning', 'path_planning_node'])
        self.get_logger().info('Node restart command issued')

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset_node_callback(self, request, response):
        self.get_logger().info('Received reset_node request. Restarting node...')
        self.reset_state()
        subprocess.Popen(['ros2', 'run', 'path_planning', 'path_planning_node'])
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
