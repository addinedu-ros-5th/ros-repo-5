import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import cv2
import numpy as np
import socket
import struct
import threading
from aruco_marker_detector.utils import ARUCO_DICT

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.publisher_ = self.create_publisher(Pose, 'aruco_pose', 10)
        self.declare_parameter('server_ip', '0.0.0.0')
        self.declare_parameter('server_port', 12345)
        self.declare_parameter('k_matrix', 'calibration_matrix.npy')
        self.declare_parameter('d_coeff', 'distortion_coefficients.npy')
        self.declare_parameter('aruco_dict_type', 'DICT_5X5_100')
        
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.k_matrix_path = self.get_parameter('k_matrix').get_parameter_value().string_value
        self.d_coeff_path = self.get_parameter('d_coeff').get_parameter_value().string_value
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').get_parameter_value().string_value
        
        self.k_matrix = np.load(self.k_matrix_path)
        self.d_coeff = np.load(self.d_coeff_path)
        
        self.aruco_dict = ARUCO_DICT[self.aruco_dict_type]
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        
        self.get_logger().info('Waiting for connection...')
        self.conn, self.addr = self.server_socket.accept()
        self.get_logger().info(f'Connected by {self.addr}')
        
        self.receive_frames_thread = threading.Thread(target=self.receive_frames)
        self.receive_frames_thread.start()
        
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f"Current robot position: {self.current_pose.position.x}, {self.current_pose.position.y}")

    def receive_frames(self):
        data = b""
        payload_size = struct.calcsize(">L")

        while True:
            while len(data) < payload_size:
                packet = self.conn.recv(4096)
                if not packet:
                    break
                data += packet

            if not packet:
                break

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]

            while len(data) < msg_size:
                data += self.conn.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            pose, frame_with_markers = self.pose_estimation(frame)
            self.publisher_.publish(pose)

            cv2.imshow('Aruco Markers', frame_with_markers)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        self.conn.close()
        cv2.destroyAllWindows()

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            for i in range(len(corners)):
                marker_corners = corners[i][0]
                marker_id = ids[i][0]

                # Draw markers
                cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f"ID: {marker_id}", (marker_corners[0][0], marker_corners[0][1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Define object points
                object_points = np.array([[2.2, -0.06, 0.15], [2.2, 0.06, 0.15], [2.2, 0.06, 0.03], [2.2, -0.06, 0.03], [2.2, 0., 0.09]], dtype=np.float32)

                image_points = np.array([
                    marker_corners[0], 
                    marker_corners[1], 
                    marker_corners[2], 
                    marker_corners[3],
                    [marker_corners[0][0] + (marker_corners[2][0] - marker_corners[0][0]) / 2,
                    marker_corners[0][1] + (marker_corners[2][1] - marker_corners[0][1]) / 2]
                ], dtype=np.float32)

                _, rvec, tvec = cv2.solvePnP(object_points, image_points, self.k_matrix, self.d_coeff)

                pose = Pose()

                if self.current_pose:
                    pose.position.x = self.current_pose.position.x + tvec[0][0]
                    pose.position.y = self.current_pose.position.y + tvec[1][0]
                    pose.position.z = tvec[2][0]
                    
                    pose.orientation.x = rvec[0][0]
                    pose.orientation.y = rvec[1][0]
                    pose.orientation.z = rvec[2][0]

                return pose, frame

        return Pose(), frame

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
