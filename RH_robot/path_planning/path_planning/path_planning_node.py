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

        self.subscription = self.create_subscription(
            String,
            '/send_park_num_23',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10)
        
        self.send_park_num_publisher = self.create_publisher(String, '/send_park_num', 10)
        self.robot_status_publisher = self.create_publisher(String, '/robot_status', 10)
        self.amcl_pose = Pose()
        
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
            'RI1TB2': ['HI1TB2ON'],
            'GI1TB2ON': ['SI1TB2', 'HB2TR1OFF'],
            'RI1TE2': ['HI1TE2ON'],
            'GI1TE2ON': ['SI1TE2', 'HE2TR1OFF'],
            'RB2TG1': ['HB2TG1ON'],
            'GB2TG1ON': ['SB2TG1'],
            'RG1TF1': ['HF1TR1OFF'],
            'RE2TA2': ['HE2TA2ON'],
            'RA2TG1': ['HA2TG1ON'],
            'GA2TG1ON': ['SA2TG1'],
            'GE2TA22': ['HA2TR1OFF'],
            'RG1TO1': ['SG1TO1', 'HO1TR1OFF']
        }

        
        self.busy_signals = ['RI1TB2', 'RI1TE2', 'RB2TG1', 'RE2TA2', 'RA2TG1']
        self.available_signals = ['GB2TR1OFF', 'GE2TR1OFF', 'GF1TR1OFF', 'GO1TR1OFF', 'GA2TR1OFF']

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

    def robot_check_callback(self, msg):
        self.get_logger().info(f'Robot check received: "{msg.data}"')
        # Receive the message from /robot_check and publish it to /send_park_num
        self.publish_to_send_park_num(msg.data)

    def publish_robot_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.robot_status_publisher.publish(status_msg)
        self.get_logger().info(f'Published robot status: {status}')

    def publish_to_send_park_num(self, data):
        park_num_msg = String()
        park_num_msg.data = data
        self.send_park_num_publisher.publish(park_num_msg)
        self.get_logger().info(f'Published to /send_park_num: {data}')

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def get_current_pose(self):
        return self.amcl_pose

    def navigate_to_waypoints(self):
        if not self.navigation_active:
            return
        
        if self.current_waypoint_index >= len(self.current_waypoints):
            self.navigation_active = False
            
            if self.current_route in ['Hello World!']:
                self.get_logger().info('Hi')

            elif self.current_route in ['GI1TB2ON', 'RB2TG1', 'RG1TF1']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GI1TB2ON, RB2TG1, RG1TF1')
                    self.get_logger().info('돌아라')
                    self.perform_post_navigation_action_right()
                    self.get_logger().info('좀')
                    self.post_action_performed = True

            elif self.current_route in ['RA2TG1']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RA2TG1')
                    self.perform_post_navigation_action_left()
                    self.post_action_performed = True
                    self.get_logger().info(f'Post action performed: {self.post_action_performed}')

            elif self.current_route in ['GA2TG1ON']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GA2TG1ON')
                    self.publish_task_completed1()
                    self.post_action_performed = True
                    self.get_logger().info(f'publish_task_completed1')

            elif self.current_route in ['GE2TA22']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GE2TA22')
                    self.perform_post_navigation_action_left()
                    self.post_action_performed = True
                    self.get_logger().info(f'Post action performed: {self.post_action_performed}')

            elif self.current_route in ['GE2TA2ON']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GE2TA2ON')
                    self.perform_post_navigation_action_curve()
                    self.post_action_performed = True

            elif self.current_route in ['GI1TE2ON', 'RE2TA2']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for GI1TE2ON, RE2TA2')
                    self.perform_post_navigastion_action_right2()
                    self.post_action_performed = True

            elif self.current_route in ['RI1TB2', 'GB2TR1OFF', 'RI1TE2', 'GE2TR1OFF', 'GF1TR1OFF', 'GA2TR1OFF', 'GO1TR1OFF']:
                if not self.post_action_performed:
                    self.get_logger().info('Performing post action for RI1TB2, GB2TR1OFF, RI1TE2, GE2TR1OFF, GF1TR1OFF, GA2TR1OFF, GO1TR1OFF')
                    self.perform_post_navigation_action_right_180()
                    self.post_action_performed = True

            # 모든 경우에서 gogo_signal_to_park_num에 따라 메시지 발행
            if self.current_route in self.gogo_signal_to_park_num:
                if not self.tg1_published:
                    park_num_signals = self.gogo_signal_to_park_num[self.current_route]
                    for signal in park_num_signals:
                        self.get_logger().info(f'Publishing {signal} to /send_park_num')
                        msg = String()
                        msg.data = signal
                        self.send_park_num_publisher.publish(msg)
                    self.tg1_published = True

            return

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
        self.rotate_for_time_left(3)

        self.move_backward_for_time(5)
        self.post_action_performed = True
        self.reset_state()
        self.publish_task_completed()

    def publish_task_completed(self):
        msg = String()
        msg.data = 'task_completed'
        self.send_park_num_publisher.publish(msg)  # Modify to publish on /send_park_num
        self.get_logger().info('Published task_completed')

    def publish_task_completed1(self):
        msg = String()
        msg.data = 'task_completed1'
        self.send_park_num_publisher.publish(msg)  # Modify to publish on /send_park_num
        self.get_logger().info('Published task_completed1')

    def perform_post_navigation_action_right(self):
        self.rotate_for_time_right(2.1)  # 90도 회전
        self.get_logger().info('왜 안 돌아')
        self.move_backward_for_time(5)
        self.post_action_performed = True
        self.reset_state()

    def perform_post_navigation_action_left(self):
        self.rotate_for_time_left(2.1)

        self.move_backward_for_time(7)
        self.post_action_performed = True
        self.reset_state()
        self.get_logger().info(f'후속 완료2')

    def perform_post_navigation_action_right_180(self):
        self.rotate_for_time_right(5)  # 180도 회전

        self.post_action_performed = True
        self.reset_state()

    def perform_post_navigastion_action_right2(self):
        self.rotate_for_time_right(2.5)

        self.post_action_performed = True
        self.reset_state()

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

    def rotate_for_right(self, duration):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = -0.2
        self.publish_command_for_duration(cmd, duration)

    def move_backward_for_time(self, time):
        cmd = Twist()
        cmd.linear.x = -0.1
        self.publish_command_for_duration(cmd, time)

    def publish_command_for_duration(self, cmd, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.publisher_.publish(cmd)
        self.publisher_.publish(Twist())

    def reset_state(self):
        self.reached_waypoint = False
        self.navigation_active = False
        self.tg1_published = False
        self.post_action_performed = False
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.get_logger().info(f'후속완료1 초기화 x')

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def reset_node_callback(self, request, response):
        self.get_logger().info('Received reset_node request. Resetting state...')
        self.reset_state()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
