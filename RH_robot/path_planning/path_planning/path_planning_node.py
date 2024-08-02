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

        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(1.0, 0.0, 0.1)
        self.last_time = self.get_clock().now()
        self.current_route = ""
        self.tg1_published = False
        self.post_action_performed = False
        self.srv = self.create_service(Empty, 'reset_node', self.reset_node_callback)

        self.gogo_signal_to_park_num = {
            'GA2TG1': 'SA2TG1',
            'GB2TG1': 'SB2TG1',
            'RG1TF1': 'SG1TF1',
            'RG1TO1': 'SG1TO1',
            'RF1TO1': 'SF1TO1',
            'GE2TA22': 'SE2TA2',
            'RI1TB2': 'HI1000',
            'GI1TB2': 'HB2000',
        }
        
        # 작업 중인 상태로 설정할 신호 목록
        self.busy_signals = ['RB2TG1', 'RI1TB2', 'RF1TO1', 'RA2TG1', 'RE2TA2']

        # 작업 가능 상태로 설정할 신호 목록
        self.available_signals = ['GG1TF1', 'GG1TO1', 'GF1TO1', 'GB2TR1']

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

        _, routes = self.waypoints_dict
        if data in routes:
            self.get_logger().info(f'Starting waypoint navigation for route {data}')
            waypoint_indices = routes[data]
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = data
            self.tg1_published = False
            self.post_action_performed = False
        elif data in self.gogo_signal_to_park_num:
            self.get_logger().info(f'Starting waypoint navigation for GOGO route {data}')
            waypoint_indices = list(range(1, 5))
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
            self.current_route = data
            self.tg1_published = False
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
            self.get_logger().info('HELLO')
            return
        
        if self.current_waypoint_index >= len(self.current_waypoints):
            self.get_logger().info('All waypoints reached')
            

            # 현재 경로 출력
            self.get_logger().info(f'Current route: {self.current_route}')
            self.get_logger().info(f'Post action performed: {self.post_action_performed}')


            # 현재 경로에 따라 후속 작업 처리
            if self.current_route in ['RB2TG1', 'RG1TF1', 'GI1TB2']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RB2TG1, RG1TF1, GI1TB2')
                    self.perform_post_navigation_action_right()
                    self.post_action_performed = True

            elif self.current_route in ['RA2TG1', 'GE2TA22']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RA2TG1, GE2TA22')
                    self.perform_post_navigation_action_left()
                    self.post_action_performed = True

            elif self.current_route in ['GE2TA2']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GE2TA2')
                    self.perform_post_navigation_action_curve()
                    self.post_action_performed = True

            elif self.current_route in ['RF1TO1', 'RE2TA2']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RF1TO1, RE2TA2')
                    self.perform_post_navigastion_action_right2()
                    self.post_action_performed = True

            elif self.current_route in ['RI1TB2', 'GG1TO1', 'GG1TF1', 'GB2TR1']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RI1TB2, GG1TO1, GG1TF1, GB2TR1')
                    self.perform_post_navigation_action_right_180()
                    self.post_action_performed = True

            elif self.current_route in self.gogo_signal_to_park_num:
                if not self.tg1_published:
                    park_num_signal = self.gogo_signal_to_park_num[self.current_route]
                    self.get_logger().info(f'Publishing {park_num_signal} to /send_park_num')
                    msg = String()
                    msg.data = park_num_signal
                    self.send_park_num_publisher.publish(msg)
                    self.tg1_published = True

            self.navigation_active = False
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

    def perform_post_navigation_action_curve(self):
        self.get_logger().info(f'Curve')
        self.rotate_for_right(2)
        rclpy.spin_once(self, timeout_sec=2)
        self.get_logger().info(f'left')
        self.rotate_for_time_left(4)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    def perform_post_navigation_action_right(self):
        self.rotate_for_time_right(2.1)  # 90도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.move_backward_for_time(5)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node() 

    def perform_post_navigation_action_left(self):
        self.rotate_for_time_left(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.move_backward_for_time(7)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    def perform_post_navigation_action_right_180(self):
        self.rotate_for_time_right(5)  # 180도 회전
        rclpy.spin_once(self, timeout_sec=2)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node() 

    def perform_post_navigastion_action_right2(self):
        self.rotate_for_time_right(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    # 우회전 
    def perform_post_navigation_action_rotation_right_90(self):
        self.rotate_for_right(2.1)
        rclpy.spin_once(self, timeout_sec=2)
        self.post_action_performed = True
        self.reset_state()
        self.restart_node()

    def rotate_for_time_left(self, angle):
        cmd = Twist()
        cmd.angular.z = 0.5
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
        cmd.linear.x = 0.1
        cmd.angular.z = -0.2  # 각속도 설정 (음수는 우회전, 양수는 좌회전)
        self.publish_command_for_duration(cmd, duration)

    def move_backward_for_time(self, time):
        cmd = Twist()
        cmd.linear.x = -0.1
        self.publish_command_for_duration(cmd, time)

    def publish_command_for_duration(self, cmd, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.publisher_.publish(Twist())  # 멈춤 명령 발행

    def reset_state(self):
        self.reached_waypoint = False
        self.navigation_active = False
        self.tg1_published = False
        self.post_action_performed = False
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.current_route = ""

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