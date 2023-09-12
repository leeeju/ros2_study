import socket

host = '192.168.0.78'
port = 9000

# 서버 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # port usage check
server_socket.bind((host, port))
server_socket.listen(1)

print(f"서버가 {host}:{port}에서 시작되었습니다.")

while True:
    client_socket, client_address = server_socket.accept()
    print(f"클라이언트 {client_address}가 연결되었습니다.")

    data = client_socket.recv(1024).decode()
    print(f"클라이언트로부터 받은 데이터: {data}")

    response = f"서버에서 받은 데이터: {data}"
    client_socket.send(response.encode())

    client_socket.close()