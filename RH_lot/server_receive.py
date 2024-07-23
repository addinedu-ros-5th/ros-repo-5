import socket
import threading
def handle_client(client_socket):
    while True:
        try:
            message = client_socket.recv(1024).decode('utf-8').strip()
            if not message:
                break
            if message.startswith("MATCH:"):
                print(f"Matching card detected: {message[6:]}")
            elif message.startswith("NOMATCH:"):
                print(f"Non-matching card detected: {message[8:]}")
        except Exception as e:
            print(f"Error: {e}")
            break
    client_socket.close()
def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 8080)) #
    server.listen(5)
    print("Server started on port 8080")
    while True:
        client, addr = server.accept()
        print(f"Accepted connection from {addr[0]}:{addr[1]}")
        client_handler = threading.Thread(target=handle_client, args=(client,))
        client_handler.start()
if __name__ == "__main__":
    start_server()