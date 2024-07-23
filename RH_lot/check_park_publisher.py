import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class CheckPublisher(Node):
    def __init__(self):
        super().__init__('check_park_publisher')
        self.plate_publisher = self.create_publisher(String, 'check_park_topic', 10)
        
        # Create a background thread to run the TCP server
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()

    def start_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Optionally reuse address
        server.bind(('0.0.0.0', 12346))
        server.listen(5)
        self.get_logger().info("Server started on port 12346")
        
        try:
            while True:
                client, addr = server.accept()
                self.get_logger().info(f"Accepted connection from {addr[0]}:{addr[1]}")
                client_handler = threading.Thread(target=self.handle_client, args=(client,))
                client_handler.start()
        except KeyboardInterrupt:
            self.get_logger().info("Server shutting down...")
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")
        finally:
            server.close()
            self.get_logger().info("Server closed")

    def handle_client(self, client_socket):
        try:
            while True:
                message = client_socket.recv(1024).decode('utf-8').strip()
                if not message:
                    break
                
                if message.startswith("MATCH:"):
                    plate_number = message[6:]
                    self.publish_plate(plate_number)
                elif message.startswith("NOMATCH:"):
                    plate_number = message[8:]
                    self.publish_plate(plate_number)
                else:
                    self.get_logger().warn(f"Unrecognized message: {message}")

        except Exception as e:
            self.get_logger().error(f"Client handling error: {e}")
        finally:
            try:
                client_socket.close()
            except Exception as e:
                self.get_logger().error(f"Error closing client socket: {e}")

    def publish_plate(self, plate_number):
        plate_msg = String()
        plate_msg.data = plate_number
        self.plate_publisher.publish(plate_msg)
        self.get_logger().info(f'Published plate: {plate_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CheckPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
