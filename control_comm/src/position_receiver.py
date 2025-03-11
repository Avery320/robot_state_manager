#!/usr/bin/env python3
import socket

HOST = '0.0.0.0'  # 伺服器監聽的 IP
PORT = 65432           # 與發送端設定一致的埠號

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((HOST, PORT))
        server.listen()
        print(f"Server listening on {HOST}:{PORT}")
        
        conn, addr = server.accept()
        with conn:
            print("Connected by", addr)
            buffer = ""
            while True:
                data = conn.recv(1024)
                if not data:
                    print("Connection closed by client.")
                    break
                raw_data = data.decode('utf-8')
                # print("Raw data received:", raw_data)
                buffer += raw_data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    # 擷取第一個 "(" 與最後一個 ")" 之間的內容（包含括號）
                    start = line.find('(')
                    end = line.rfind(')')
                    if start != -1 and end != -1 and end > start:
                        tuple_str = line[start:end+1]
                        # print("Received tuple data:", tuple_str)
                        print(tuple_str)
                    else:
                        print("Tuple not found in line:", line)

if __name__ == '__main__':
    main()
