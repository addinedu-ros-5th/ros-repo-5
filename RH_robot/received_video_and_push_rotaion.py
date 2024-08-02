import cv2
import socket
import struct
import threading
import numpy as np
import argparse
import sys
from utils import ARUCO_DICT
import time

# ArUco 마커 감지 및 방향 표시 함수
def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    message = None
    marker_detected = False  # 마커 감지 여부 플래그

    if ids is not None and len(ids) > 0:
        marker_detected = True  # 마커가 감지된 경우 플래그 업데이트
        for idx, id_ in enumerate(ids):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], 0.02, matrix_coefficients, distortion_coefficients)

            # Draw Axis
            axis_len = 0.1  # Length of the axis
            axis_points = np.float32([[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, -axis_len]]).reshape(-1, 3, 1)
            axis_points_img, _ = cv2.projectPoints(axis_points, rvec, tvec, matrix_coefficients, distortion_coefficients)

            # Extract center point of the marker
            cx = int((corners[idx][0][0][0] + corners[idx][0][1][0] + corners[idx][0][2][0] + corners[idx][0][3][0]) / 4)
            cy = int((corners[idx][0][0][1] + corners[idx][0][1][1] + corners[idx][0][2][1] + corners[idx][0][3][1]) / 4)

            frame = draw_axis(frame, (cx, cy), axis_points_img)

            # Convert coordinates to be centered at the middle of the frame
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            cx_centered = cx - frame_center_x
            cy_centered = cy - frame_center_y

            # Draw ID and center point
            cv2.putText(frame, str(ids[idx][0]), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

            # Determine position and give direction
            if cx_centered < -20:  # Marker is to the left of center
                message = "L"
            elif cx_centered > 20:  # Marker is to the right of center
                message = "R"
            elif cy_centered < -20:  # Marker is above the center
                message = "U"
            elif cy_centered > 20:  # Marker is below the center
                message = "D"
            else:  # Marker is centered
                message = "C"

            if message:
                cv2.putText(frame, message, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    if not marker_detected:
        # 마커가 감지되지 않은 경우 처리 (예: 이전 방향을 유지하거나, 특정 동작 수행)
        message = "C"  # 임시로 중앙으로 설정

    return frame, message

# Draw axis function
def draw_axis(frame, marker_center, axis_points_img):
    cx, cy = marker_center

    # Draw axis lines from the marker center
    for i in range(1, 4):
        p1 = (cx, cy)
        p2 = (int(axis_points_img[i][0][0]), int(axis_points_img[i][0][1]))
        color = (0, 0, 255) if i == 1 else (0, 255, 0) if i == 2 else (255, 0, 0)
        cv2.line(frame, p1, p2, color, 2)

    return frame

# Receive frames function
def receive_frames(conn, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    data = b""
    payload_size = struct.calcsize(">L")

    while True:
        while len(data) < payload_size:
            packet = conn.recv(4096)
            if not packet:
                break
            data += packet

        if not packet:
            break

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += conn.recv(4096)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        # Perform ArUco marker detection and pose estimation
        output, message = pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients)

        cv2.imshow('Estimated Pose', output)

        if message:
            conn.sendall(message.encode('utf-8'))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    conn.close()
    cv2.destroyAllWindows()

# Server function
def server(aruco_dict_type, matrix_coefficients, distortion_coefficients):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(1)

    print("Waiting for connection...")
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    receive_frames_thread = threading.Thread(target=receive_frames, args=(conn, aruco_dict_type, matrix_coefficients, distortion_coefficients))
    receive_frames_thread.start()
    receive_frames_thread.join()

    server_socket.close()

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    k = np.load(args["K_Matrix"])
    d = np.load(args["D_Coeff"])

    SERVER_IP = '0.0.0.0'
    SERVER_PORT = 12345

    server_thread = threading.Thread(target=server, args=(aruco_dict_type, k, d))
    server_thread.start()
    server_thread.join()
