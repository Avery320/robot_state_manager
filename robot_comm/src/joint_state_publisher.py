#!/usr/bin/env python3
import rospy
import tf2_ros
import socket
import threading

# TCP 傳輸設定：請根據實際環境修改
TCP_HOST = '172.20.10.14'  # 目標接收端 IP（例如 Grasshopper 或中介主機）
TCP_PORT = 65432           # 與接收端設定一致的埠號

# 建立全域 TCP 客戶端 socket 與連線狀態
tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_connected = False

def connect_tcp():
    global tcp_connected
    while not tcp_connected and not rospy.is_shutdown():
        try:
            tcp_client.connect((TCP_HOST, TCP_PORT))
            tcp_connected = True
            rospy.loginfo("TCP 連線成功!")
        except socket.error as e:
            rospy.logwarn("TCP 連線失敗，重試中... (%s)", e)
            rospy.sleep(1)

# 啟動背景執行緒以建立 TCP 連線
tcp_thread = threading.Thread(target=connect_tcp)
tcp_thread.daemon = True
tcp_thread.start()

def main():
    global tcp_connected  # 使用全域變數
    rospy.init_node('end_effector_tcp_sender', anonymous=True)
    
    # 初始化 tf2 Buffer 與 Listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10)  # 每秒更新 10 次
    while not rospy.is_shutdown():
        try:
            # 嘗試取得從 base_link 到 link_6 的轉換資訊
            transform = tf_buffer.lookup_transform("base_link", "link_6", rospy.Time(0), rospy.Duration(1.0))
            
            # 從轉換中解析出末端位置
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # 格式化成指定格式，並加上換行符號以便接收端分隔訊息
            message = f"({x}, {y}, {z})\n"
            
            # 傳送資料
            if tcp_connected:
                try:
                    tcp_client.sendall(message.encode('utf-8'))
                    rospy.loginfo("傳送資料: %s", message.strip())
                except Exception as e:
                    rospy.logerr("傳送資料失敗: %s", e)
                    tcp_client.close()
                    tcp_connected = False
                    threading.Thread(target=connect_tcp, daemon=True).start()
            else:
                rospy.logwarn("尚未建立 TCP 連線")
        except Exception as e:
            rospy.logwarn("無法獲取 TF 轉換: %s", e)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
