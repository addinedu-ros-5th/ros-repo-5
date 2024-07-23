import subprocess
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import socket
import threading

def connect_to_wifi(ssid, password):
    try:
        # Create a connection profile
        subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password], check=True)
        print(f"Connected to Wi-Fi network: {ssid}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to connect to Wi-Fi: {e}")
        raise

class CheckPublisher(Node):
    def __init__(self):
        super().__init__('check_park_publisher')
        self.plate_publisher = self.create_publisher(String, 'check_park_topic', 10)
        self.get_logger().info('CheckPublisher node has been started.')

    def publish_index(self, index_value):
        msg = String()
        msg.data = index_value
        self.plate_publisher.publish(msg)
        self.get_logger().info(f'Published INDEX value: {index_value}')

def publish_to_topic(publisher, index_value):
    publisher.publish_index(index_value)

def handle_client(client_socket, publisher):
    while True:
        try:
            message = client_socket.recv(1024).decode('utf-8').strip()
            if not message:
                break
            if message.startswith("MATCH:"):
                print(f"Matching card detected: {message[6:]}")
                index_part = message.split(',INDEX:')
                if len(index_part) == 2:
                    index_value = index_part[1].strip()
                    threading.Thread(target=publish_to_topic, args=(publisher, index_value)).start()
            elif message.startswith("NOMATCH:"):
                print(f"Non-matching card detected: {message[8:]}")
        except Exception as e:
            print(f"Error: {e}")
            break
    client_socket.close()

def start_server(publisher):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 8080))
    server.listen(5)
    print("Server started on port 8080")
    while True:
        client, addr = server.accept()
        print(f"Accepted connection from {addr[0]}:{addr[1]}")
        client_handler = threading.Thread(target=handle_client, args=(client, publisher))
        client_handler.start()

def main():
    ssid = "ros2final5"
    password = "00000000"

    try:
        connect_to_wifi(ssid, password)
    except Exception as e:
        print(f"Error during Wi-Fi connection: {e}")
        return

    rclpy.init()
    publisher = CheckPublisher()

    try:
        threading.Thread(target=start_server, args=(publisher,)).start()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
