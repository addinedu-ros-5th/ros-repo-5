import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
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
        self.aruco_subscription = self.create_subscription(Pose, '/aruco_pose', self.aruco_callback, 10)
        
        self.aruco_pose = Pose()

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
            self.current_waypoints = [self.waypoints_dict[0][i - 1] for i in waypoint_indices]  # Waypoint 인덱스는 1부터 시작하므로 1을 뺌
            self.current_waypoint_index = 0
            self.reached_waypoint = False
            self.navigation_active = True
        else:
            self.get_logger().info(f'Invalid route: {msg.data}')

    def aruco_callback(self, msg):
        self.aruco_pose = msg

    def get_current_pose(self):
        return self.aruco_pose

    def navigate_to_waypoints(self):
        if not self.navigation_active:
            return
        
        if self.current_waypoint_index >= len(self.current_waypoints):
            self.get_logger().info('All waypoints reached')
            self.navigation_active = False
            return

        goal = self.current_waypoints[self.current_waypoint_index]
        goal_x, goal_y, goal_yaw = goal['x'], goal['y'], goal['z']
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
            current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)
            angle_diff = self.normalize_angle(goal_yaw - current_yaw)

            if abs(angle_diff) < self.angle_tolerance:
                self.get_logger().info(f'Orientation at waypoint {self.current_waypoint_index} achieved')

                self.current_waypoint_index += 1
                self.reached_waypoint = False
            else:
                cmd = Twist()
                cmd.angular.z = self.angular_pid.compute(angle_diff, 0, dt)
                self.publisher_.publish(cmd)
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
