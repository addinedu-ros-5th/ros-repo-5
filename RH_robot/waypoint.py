import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

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
        super().__init__('waypoint')
        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/base_controller/odom', self.odom_callback, 10)
        self.amcl_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        self.odom_pose = Pose()
        self.amcl_pose = Pose()  # Initialize amcl_pose
        self.waypoints = [
            (0.7, 0.1, -1),
            (0.7, -1.5, -2),
            (-1., -1.5, -3),
        ]
        self.current_waypoint_index = 0
        self.tolerance = 0.1
        self.angle_tolerance = 0.05
        self.timer = self.create_timer(0.1, self.navigate_to_waypoints)
        self.reached_waypoint = False

        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(1.0, 0.0, 0.1)

        self.last_time = self.get_clock().now()
        self.last_amcl_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose
        self.last_amcl_time = self.get_clock().now()

    def get_current_pose(self):
        # Use AMCL pose if the last update was within 1 second, otherwise use odom pose
        current_time = self.get_clock().now()
        if (current_time - self.last_amcl_time).nanoseconds / 1e9 < 1.0:
            return self.amcl_pose
        else:
            return self.odom_pose

    def navigate_to_waypoints(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached')
            return

        goal_x, goal_y, goal_yaw = self.waypoints[self.current_waypoint_index]
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

                if self.current_waypoint_index == len(self.waypoints) - 1:
                    # 마지막 웨이포인트에 도달했을 때 후진
                    reverse_duration = 2.0  # 후진 시간 (예: 2초)
                    reverse_speed = -0.2  # 후진 선속도 (예: -0.2m/s)
                    start_time = self.get_clock().now()

                    while (self.get_clock().now() - start_time).nanoseconds / 1e9 < reverse_duration:
                        cmd = Twist()
                        cmd.linear.x = reverse_speed
                        cmd.angular.z = 0.0  # 각속도는 0으로 설정하여 직진 후진만 수행
                        self.publisher_.publish(cmd)
                        rclpy.spin_once(self)

                    self.get_logger().info('Reverse motion completed')
                else:
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
