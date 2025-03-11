#!/usr/bin/env python
import rospy
import socket
import json
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String, ColorRGBA
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import time

def tcp_server():
    """建立TCP伺服器並等待接收點位資訊"""
    HOST = '0.0.0.0'  # 監聽所有網路介面
    PORT = 65432
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 添加 socket 選項，允許重用地址
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        rospy.loginfo("等待來自控制節點的連線，監聽 port %d...", PORT)
        conn, addr = s.accept()
        with conn:
            rospy.loginfo("連線來自 %s", addr)
            data = conn.recv(4096)  # 增加接收緩衝區大小
            if not data:
                return None
            try:
                received_data = json.loads(data.decode('utf-8'))
                # 回傳確認訊息
                conn.sendall(json.dumps({"status": "success", "message": "已接收點位資訊"}).encode('utf-8'))
                
                # 檢查是否收到點位數組
                if "points" in received_data:
                    return received_data["points"]
                else:
                    # 兼容舊格式，單個點位
                    return [received_data]
            except Exception as e:
                rospy.logerr("JSON 解析錯誤: %s", e)
                return None
    except OSError as e:
        if e.errno == 98:  # Address already in use
            rospy.logerr("端口 %d 已被佔用，請嘗試關閉使用該端口的程序或使用其他端口", PORT)
            rospy.sleep(5)  # 等待5秒後重試
            return None
        else:
            rospy.logerr("Socket 錯誤: %s", e)
            return None
    finally:
        # 確保 socket 被關閉
        if 's' in locals() and s:
            s.close()

def visualize_trajectory(points, marker_pub):
    """在RViz中可視化整個軌跡"""
    # 創建一個MarkerArray來顯示所有點位
    marker_array = MarkerArray()
    
    # 添加點位標記
    for i, point in enumerate(points):
        # 創建點位標記
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory_points"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.get("x", 0.0)
        marker.pose.position.y = point.get("y", 0.0)
        marker.pose.position.z = point.get("z", 0.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 小球直徑
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # 根據點的順序設置漸變顏色
        progress = float(i) / max(1, len(points) - 1)
        marker.color.r = 1.0 - progress
        marker.color.g = progress
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker_array.markers.append(marker)
    
    # 創建連接點位的線段
    line_marker = Marker()
    line_marker.header.frame_id = "base_link"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "trajectory_line"
    line_marker.id = len(points)
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.scale.x = 0.02  # 線寬
    line_marker.color.r = 0.0
    line_marker.color.g = 0.0
    line_marker.color.b = 1.0
    line_marker.color.a = 1.0
    
    # 添加所有點到線段中
    for point in points:
        p = Point()
        p.x = point.get("x", 0.0)
        p.y = point.get("y", 0.0)
        p.z = point.get("z", 0.0)
        line_marker.points.append(p)
    
    marker_array.markers.append(line_marker)
    
    # 發布MarkerArray
    marker_pub.publish(marker_array)
    rospy.loginfo("已在RViz中可視化軌跡，共 %d 個點位", len(points))

def move_arm_to_points(points, arm_move_group, status_pub, marker_pub):
    """控制機械手臂依次移動到所有點位"""
    try:
        # 設置末端執行器
        arm_move_group.set_end_effector_link("link_6")
        
        # 先可視化整個軌跡
        visualize_trajectory(points, marker_pub)
        
        # 依次移動到每個點位
        all_points_processed = True  # 追蹤是否所有點位都被成功處理
        
        for i, point in enumerate(points):
            rospy.loginfo("正在處理點位 %d/%d: %s", i+1, len(points), point)
            status_pub.publish(f"正在移動到點位 {i+1}/{len(points)}")
            
            # 創建目標姿態
            target_pose = Pose()
            target_pose.position.x = point.get("x", 0.0)
            target_pose.position.y = point.get("y", 0.0)
            target_pose.position.z = point.get("z", 0.0)
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 1.0
            
            # 設置目標姿態
            arm_move_group.set_pose_target(target_pose)
            
            # 規劃並執行移動
            rospy.loginfo("規劃link_6移動到點位 %d/%d", i+1, len(points))
            
            try:
                # 修正 plan() 方法的處理方式
                plan_result = arm_move_group.plan()
                
                # 處理不同版本的MoveIt返回值
                move_success = False
                
                # 檢查返回值類型並適當處理
                if isinstance(plan_result, tuple):
                    # 新版本 MoveIt 可能返回 (success, trajectory, planning_time, error_code)
                    if len(plan_result) == 4:  # 返回4個值
                        success, plan_trajectory, _, _ = plan_result
                        if success:
                            try:
                                rospy.loginfo("規劃成功，執行移動...")
                                arm_move_group.execute(plan_trajectory)
                                move_success = True
                            except Exception as exec_error:
                                rospy.logerr("執行移動時出錯: %s", exec_error)
                                status_pub.publish(f"執行點位 {i+1}/{len(points)} 時出錯，繼續下一個點位")
                                all_points_processed = False
                        else:
                            rospy.logerr("無法規劃到點位 %d/%d", i+1, len(points))
                            status_pub.publish(f"無法規劃到點位 {i+1}/{len(points)}，繼續下一個點位")
                            all_points_processed = False
                    elif len(plan_result) == 2:  # 返回2個值
                        success, plan_trajectory = plan_result
                        if success:
                            try:
                                rospy.loginfo("規劃成功，執行移動...")
                                arm_move_group.execute(plan_trajectory)
                                move_success = True
                            except Exception as exec_error:
                                rospy.logerr("執行移動時出錯: %s", exec_error)
                                status_pub.publish(f"執行點位 {i+1}/{len(points)} 時出錯，繼續下一個點位")
                                all_points_processed = False
                        else:
                            rospy.logerr("無法規劃到點位 %d/%d", i+1, len(points))
                            status_pub.publish(f"無法規劃到點位 {i+1}/{len(points)}，繼續下一個點位")
                            all_points_processed = False
                    else:  # 其他情況
                        rospy.logwarn("未知的plan()返回格式，嘗試直接執行...")
                        try:
                            arm_move_group.execute(plan_result[0])  # 嘗試使用第一個元素作為軌跡
                            move_success = True
                        except Exception as exec_error:
                            rospy.logerr("執行移動時出錯: %s", exec_error)
                            status_pub.publish(f"執行點位 {i+1}/{len(points)} 時出錯，繼續下一個點位")
                            all_points_processed = False
                else:
                    # 舊版本的MoveIt或直接返回軌跡
                    if plan_result:
                        try:
                            rospy.loginfo("規劃成功，執行移動...")
                            arm_move_group.execute(plan_result)
                            move_success = True
                        except Exception as exec_error:
                            rospy.logerr("執行移動時出錯: %s", exec_error)
                            status_pub.publish(f"執行點位 {i+1}/{len(points)} 時出錯，繼續下一個點位")
                            all_points_processed = False
                    else:
                        rospy.logerr("無法規劃到點位 %d/%d", i+1, len(points))
                        status_pub.publish(f"無法規劃到點位 {i+1}/{len(points)}，繼續下一個點位")
                        all_points_processed = False
                
                # 無論成功與否，都清理目標
                arm_move_group.stop()
                arm_move_group.clear_pose_targets()
                
                # 更新當前點位的標記顏色
                update_marker = Marker()
                update_marker.header.frame_id = "base_link"
                update_marker.header.stamp = rospy.Time.now()
                update_marker.ns = "trajectory_points"
                update_marker.id = i
                update_marker.type = Marker.SPHERE
                update_marker.action = Marker.MODIFY
                update_marker.pose.position.x = point.get("x", 0.0)
                update_marker.pose.position.y = point.get("y", 0.0)
                update_marker.pose.position.z = point.get("z", 0.0)
                update_marker.pose.orientation.w = 1.0
                update_marker.scale.x = 0.05
                update_marker.scale.y = 0.05
                update_marker.scale.z = 0.05
                
                # 根據移動是否成功設置不同顏色
                if move_success:
                    # 綠色表示成功到達
                    update_marker.color.r = 0.0
                    update_marker.color.g = 1.0
                    update_marker.color.b = 0.0
                else:
                    # 紅色表示無法到達
                    update_marker.color.r = 1.0
                    update_marker.color.g = 0.0
                    update_marker.color.b = 0.0
                update_marker.color.a = 1.0
                
                marker_array = MarkerArray()
                marker_array.markers.append(update_marker)
                marker_pub.publish(marker_array)
                
                if move_success:
                    status_pub.publish(f"已到達點位 {i+1}/{len(points)}")
                else:
                    status_pub.publish(f"無法到達點位 {i+1}/{len(points)}，但已標記")
                
                # 確保在點位之間有足夠的停留時間，但不要在最後一個點位後停留太久
                if i < len(points) - 1:
                    rospy.sleep(0.5)  # 短暫停留
                
            except Exception as point_error:
                rospy.logerr("處理點位 %d/%d 時出錯: %s", i+1, len(points), point_error)
                status_pub.publish(f"處理點位 {i+1}/{len(points)} 時出錯，繼續下一個點位")
                all_points_processed = False
                
                # 更新為紅色標記
                update_marker = Marker()
                update_marker.header.frame_id = "base_link"
                update_marker.header.stamp = rospy.Time.now()
                update_marker.ns = "trajectory_points"
                update_marker.id = i
                update_marker.type = Marker.SPHERE
                update_marker.action = Marker.MODIFY
                update_marker.pose.position.x = point.get("x", 0.0)
                update_marker.pose.position.y = point.get("y", 0.0)
                update_marker.pose.position.z = point.get("z", 0.0)
                update_marker.pose.orientation.w = 1.0
                update_marker.scale.x = 0.05
                update_marker.scale.y = 0.05
                update_marker.scale.z = 0.05
                update_marker.color.r = 1.0
                update_marker.color.g = 0.0
                update_marker.color.b = 0.0
                update_marker.color.a = 1.0
                
                marker_array = MarkerArray()
                marker_array.markers.append(update_marker)
                marker_pub.publish(marker_array)
                
                # 繼續處理下一個點位
                continue
        
        # 確保機械手臂停止移動
        arm_move_group.stop()
        
        if all_points_processed:
            status_pub.publish("已成功完成所有點位模擬")
            rospy.loginfo("已成功完成所有點位模擬")
        else:
            status_pub.publish("已完成所有點位模擬，但部分點位無法到達")
            rospy.loginfo("已完成所有點位模擬，但部分點位無法到達")
        
        return True
    except Exception as e:
        rospy.logerr("移動機械手臂時出錯: %s", e)
        status_pub.publish(f"移動機械手臂時出錯: {e}，但將繼續模擬")
        return False

def clear_all_markers(marker_pub):
    """清除RViz中的所有標記"""
    # 創建一個空的MarkerArray
    marker_array = MarkerArray()
    
    # 創建一個刪除所有點位的標記
    delete_marker = Marker()
    delete_marker.header.frame_id = "base_link"
    delete_marker.header.stamp = rospy.Time.now()
    delete_marker.ns = "trajectory_points"
    delete_marker.id = 0
    delete_marker.action = Marker.DELETEALL
    
    # 創建一個刪除軌跡線的標記
    delete_line = Marker()
    delete_line.header.frame_id = "base_link"
    delete_line.header.stamp = rospy.Time.now()
    delete_line.ns = "trajectory_line"
    delete_line.id = 0
    delete_line.action = Marker.DELETEALL
    
    marker_array.markers.append(delete_marker)
    marker_array.markers.append(delete_line)
    
    # 發布MarkerArray以刪除所有標記
    marker_pub.publish(marker_array)
    rospy.loginfo("已清除RViz中的所有標記")

def robot_control_node():
    rospy.init_node('robot_control_node')
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    status_pub = rospy.Publisher('/arm_status', String, queue_size=10)
    
    # 初始化MoveIt
    try:
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize([])
        
        # 獲取機器人模型
        robot = moveit_commander.RobotCommander()
        
        # 獲取場景對象
        scene = moveit_commander.PlanningSceneInterface()
        
        # 獲取機械手臂的move_group
        arm_group_name = "manipulator"  # 通常UR機器人的規劃組名稱是"manipulator"
        arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)
        
        # 設置末端執行器
        arm_move_group.set_end_effector_link("link_6")
        
        # 輸出當前的末端執行器和參考座標系
        rospy.loginfo("末端執行器: %s", arm_move_group.get_end_effector_link())
        rospy.loginfo("參考座標系: %s", arm_move_group.get_planning_frame())
        
        rospy.loginfo("成功初始化MoveIt，機械手臂規劃組: %s", arm_group_name)
        status_pub.publish("MoveIt初始化成功，使用link_6作為末端執行器")
    except Exception as e:
        rospy.logerr("初始化MoveIt時出錯: %s", e)
        rospy.logerr("將只顯示點位而不控制機械手臂")
        arm_move_group = None
        status_pub.publish("MoveIt初始化失敗，只能顯示點位")

    # 檢查端口是否可用
    try:
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        test_socket.bind(('0.0.0.0', 65432))
        test_socket.close()
    except OSError as e:
        if e.errno == 98:  # Address already in use
            rospy.logerr("端口 65432 已被佔用，請嘗試關閉使用該端口的程序")
            return  # 退出函數

    # 等待一次連線與資料傳入
    rospy.loginfo("等待接收點位資訊...")
    status_pub.publish("等待接收點位資訊...")
    points = tcp_server()
    
    if points is not None and len(points) > 0:
        rospy.loginfo("收到 %d 個點位資訊", len(points))
        status_pub.publish(f"收到 {len(points)} 個點位資訊")
        
        # 如果MoveIt初始化成功，控制機械手臂移動到點位
        if arm_move_group:
            status_pub.publish("正在規劃機械手臂軌跡...")
            success = move_arm_to_points(points, arm_move_group, status_pub, marker_pub)
            if not success:
                rospy.logwarn("機械手臂移動失敗，但已完成所有點位的模擬")
        else:
            # 僅可視化軌跡
            visualize_trajectory(points, marker_pub)
            status_pub.publish("已在RViz中可視化軌跡，但無法控制機械手臂")
            rospy.loginfo("已完成所有點位的可視化")
        
        # 等待幾秒鐘，確保訊息發布完成和可視化顯示
        rospy.sleep(5)  # 增加等待時間，確保所有點位都能被完整顯示
        rospy.loginfo("模擬完成，程序將退出")
        status_pub.publish("模擬完成，程序將退出")
        rospy.sleep(1)  # 確保最後的訊息能被接收
        
        # 清理資源並退出
        if arm_move_group:
            moveit_commander.roscpp_shutdown()
        
        # 強制退出程序
        import sys
        sys.exit(0)
    else:
        rospy.logerr("未收到有效的點位資訊，程序將退出")
        status_pub.publish("未收到有效的點位資訊，程序將退出")
        
        # 強制退出程序
        import sys
        sys.exit(0)

if __name__ == '__main__':
    try:
        robot_control_node()
    except rospy.ROSInterruptException:
        pass