import socket
import json

HOST = '10.16.128.52'  # 監聽所有可用介面，或指定 '192.168.1.100'
PORT = 65432      # 與 ROS 節點中設定的埠一致

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
    server.bind((HOST, PORT))
    server.listen()
    print(f"Server listening on {HOST}:{PORT}")
    
    conn, addr = server.accept()
    with conn:
        print("Connected by", addr)
        while True:
            data = conn.recv(1024)  # 根據資料大小調整緩衝區
            if not data:
                break
            try:
                # 將接收到的 JSON 數據解析為 Python 字典
                joint_data = json.loads(data.decode('utf-8'))
                print("Received Joint Data:", joint_data)
            except json.JSONDecodeError as e:
                print("JSON decode error:", e)
