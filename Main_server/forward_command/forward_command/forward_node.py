import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import json
import os

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
        self.odom_subscription = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.amcl_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        self.send_park_num_publisher = self.create_publisher(String, '/send_park_num', 10)

        self.odom_pose = Pose()
        self.amcl_pose = Pose()  # Initialize amcl_pose

        self.waypoints_dict = self.load_waypoints()
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.tolerance = 0.1
        self.angle_tolerance = 0.05
        self.timer = self.create_timer(0.1, self.navigate_to_waypoints)
        self.reached_waypoint = False
        self.navigation_active = False

        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(1.0, 0.0, 0.1)

        self.last_time = self.get_clock().now()
        self.last_amcl_time = self.get_clock().now()

        self.current_route = ""
        self.tg1_published = False  # TG1 메시지가 한 번 발행되었는지 확인하는 플래그 변수

    def load_waypoints(self):
        waypoints_file = '/home/ros2/pinkbot/src/pinklab_minibot_robot/forward_command/forward_command/waypoints.json'
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return {}
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
            self.tg1_published = False  # 새로운 작업이 시작될 때 플래그 초기화
        else:
            self.get_logger().info(f'Invalid route: {msg.data}')

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.last_amcl_time = self.get_clock().now()

    def get_current_pose(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_amcl_time).nanoseconds / 1e9 < 1.0:
            return self.amcl_pose
        else:
            return self.odom_pose

    def navigate_to_waypoints(self):
        if not self.navigation_active:
            return
        
        if self.current_waypoint_index >= len(self.current_waypoints):
            self.get_logger().info('All waypoints reached')
            self.navigation_active = False

            # 현재 경로에 따라 send_park_num 토픽에 메시지 발행
            if self.current_route in ['RA2TG1', 'RB2TG1', 'RA2TG2', 'RB2TG2']:
                if not self.tg1_published:
                    new_msg = String()
                    if self.current_route == 'RA2TG1':
                        new_msg.data = 'SA2TG1'
                    elif self.current_route == 'RB2TG1':
                        new_msg.data = 'SB2TG1'
                    elif self.current_route == 'RA2TG2':
                        new_msg.data = 'SA2TG2'
                    elif self.current_route == 'RB2TG2':
                        new_msg.data = 'SB2TG2'
                    self.send_park_num_publisher.publish(new_msg)
                    self.tg1_published = True  # 한 번만 발행되도록 설정
            elif self.current_route in ['RG1TO1', 'RG1TF1']:
                new_msg = String()
                if self.current_route == 'RG1TO1':
                    new_msg.data = 'SG1TO1'
                elif self.current_route == 'RG1TF1':
                    new_msg.data = 'SG1TF1'
                self.send_park_num_publisher.publish(new_msg)

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

        if distance < self.tolerance and not self.reached_waypoint:
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
            if abs(angle_diff) > 0.2:
                cmd.angular.z = self.angular_pid.compute(angle_diff, 0, dt)
            else:
                cmd.linear.x = self.linear_pid.compute(distance, 0, dt)
            cmd.angular.z = self.angular_pid.compute(angle_diff, 0, dt)
            self.publisher_.publish(cmd)

    def start_navigation(self, route):
        self.get_logger().info(f'Starting waypoint navigation for route {route}')
        _, routes = self.waypoints_dict
        waypoint_indices = routes[route]
        self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]
        self.current_waypoint_index = 0
        self.reached_waypoint = False
        self.navigation_active = True
        self.current_route = route

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    rclpy.spin(waypoint_navigator)
    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
