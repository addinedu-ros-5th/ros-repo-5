import subprocess
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import socket
import threading
from config import Config  # Import the Config class

def connect_to_wifi(ssid, password):
    try:
        subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password], check=True)
        print(f"Connected to Wi-Fi network: {ssid}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to connect to Wi-Fi: {e}")
        raise

class CheckPublisher(Node):
    def __init__(self):
        super().__init__('check_park_publisher')
        self.plate_publisher = self.create_publisher(String, 'check_park_23', 10) # ROBOT ID (_23)
        self.get_logger().info('CheckPublisher node has been started.')
    
    def publish_index(self, index_value):
        try:
            index = int(index_value)
            target_uid_info = Config.get_target_uid(index)
            msg = String()
            msg.data = target_uid_info["name"]
            self.plate_publisher.publish(msg)
            self.get_logger().info(f'Published INDEX value: {target_uid_info["name"]}')
        except ValueError:
            self.get_logger().error(f'Invalid index value: {index_value}')

class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')
        self.signal_publisher = self.create_publisher(String, 'lot_signal', 10)
        self.get_logger().info('SignalPublisher node has been started.')
    
    def publish_signal(self, signal):
        msg = String()
        msg.data = signal
        self.signal_publisher.publish(msg)
        self.get_logger().info(f'Published lot signal: {signal}')

def publish_index(check_publisher, index_value):
    check_publisher.publish_index(index_value)

def publish_signal(signal_publisher, signal):
    signal_publisher.publish_signal(signal)

def handle_client(client_socket, check_publisher, signal_publisher):
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
                    signal = "MATCH"
                    threading.Thread(target=publish_index, args=(check_publisher, index_value)).start()
                    threading.Thread(target=publish_signal, args=(signal_publisher, signal)).start()

            elif message.startswith("NOMATCH:"):
                print(f"Non-matching card detected: {message[8:]}")
                signal = "NOMATCH"
                threading.Thread(target=publish_signal, args=(signal_publisher, signal)).start()

            elif message.startswith("Test message from ESP32:"):
                print(f"{message}")
                signal = message
                threading.Thread(target=publish_signal, args=(signal_publisher, signal)).start()

        except Exception as e:
            print(f"Error: {e}")
            break

    client_socket.close()

def start_server(check_publisher, signal_publisher):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((Config.SERVER_IP, Config.SERVER_PORT))
    server.listen(5)
    print(f"Server started on {Config.SERVER_IP}:{Config.SERVER_PORT}")
    while True:
        client, addr = server.accept()
        print(f"Accepted connection from {addr[0]}:{addr[1]}")
        client_handler = threading.Thread(target=handle_client, args=(client, check_publisher, signal_publisher))
        client_handler.start()

def main():
    try:
        connect_to_wifi(Config.WIFI_SSID, Config.WIFI_PASSWORD)
    except Exception as e:
        print(f"Error during Wi-Fi connection: {e}")
        return

    rclpy.init()
    check_publisher = CheckPublisher()
    signal_publisher = SignalPublisher()

    try:
        threading.Thread(target=start_server, args=(check_publisher, signal_publisher)).start()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(check_publisher)
        executor.add_node(signal_publisher)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        check_publisher.destroy_node()
        signal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
