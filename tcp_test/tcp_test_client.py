import socket


server_host = '192.168.0.78'
server_port = 9000

data_to_send = "Hello, Server!"

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_host, server_port))

client_socket.send(data_to_send.encode())

response = client_socket.recv(1024).decode()
print(f"서버로부터 받은 응답: {response}")

client_socket.close()