#!/usr/bin/env python
import socket
import json
import random
import time
import argparse
import math

def generate_random_points(count=3, min_val=-0.25, max_val=0.25):
    """生成隨機的一組點位資訊"""
    points = []
    for i in range(count):
        points.append({
            "x": round(random.uniform(min_val, max_val), 3),
            "y": round(random.uniform(min_val, max_val), 3),
            "z": round(random.uniform(min_val, max_val), 3)
        })
    return points

def generate_circle_points(count=10, radius=0.2, height=0.2):
    """生成圓形軌跡的點位"""
    points = []
    for i in range(count):
        angle = 2 * math.pi * i / count
        points.append({
            "x": round(radius * math.cos(angle), 3),
            "y": round(radius * math.sin(angle), 3),
            "z": height
        })
    return points

def generate_square_points(side_length=0.5, height=0.3, points_per_side=3):
    """生成正方形軌跡的點位"""
    points = []
    half_side = side_length / 2
    
    # 生成四條邊的點位
    # 底邊 (x從-half_side到half_side, y=-half_side)
    for i in range(points_per_side):
        t = i / (points_per_side - 1)
        points.append({
            "x": round(-half_side + side_length * t, 3),
            "y": round(-half_side, 3),
            "z": height
        })
    
    # 右邊 (x=half_side, y從-half_side到half_side)
    for i in range(points_per_side):
        t = i / (points_per_side - 1)
        points.append({
            "x": round(half_side, 3),
            "y": round(-half_side + side_length * t, 3),
            "z": height
        })
    
    # 頂邊 (x從half_side到-half_side, y=half_side)
    for i in range(points_per_side):
        t = i / (points_per_side - 1)
        points.append({
            "x": round(half_side - side_length * t, 3),
            "y": round(half_side, 3),
            "z": height
        })
    
    # 左邊 (x=-half_side, y從half_side到-half_side)
    for i in range(points_per_side):
        t = i / (points_per_side - 1)
        points.append({
            "x": round(-half_side, 3),
            "y": round(half_side - side_length * t, 3),
            "z": height
        })
    
    return points

def generate_line_points(start_point, end_point, count=5):
    """在兩點之間生成等距離的點位"""
    points = []
    for i in range(count):
        t = i / (count - 1) if count > 1 else 0
        points.append({
            "x": round(start_point[0] + t * (end_point[0] - start_point[0]), 3),
            "y": round(start_point[1] + t * (end_point[1] - start_point[1]), 3),
            "z": round(start_point[2] + t * (end_point[2] - start_point[2]), 3)
        })
    return points

def send_points(points, host, port):
    """傳送一組點位資訊到指定的主機和端口"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((host, port))
            
            # 確保每個點位都有 x, y, z 座標
            for i, point in enumerate(points):
                if "x" not in point or "y" not in point or "z" not in point:
                    print(f"警告: 點位 {i+1} 缺少座標資訊，將使用預設值 0.0")
                    point["x"] = point.get("x", 0.0)
                    point["y"] = point.get("y", 0.0)
                    point["z"] = point.get("z", 0.0)
            
            # 確保發送的資料格式正確
            data = json.dumps({"points": points})
            s.sendall(data.encode('utf-8'))
            print(f"已傳送 {len(points)} 個點位資訊")
            
            # 接收回應
            try:
                s.settimeout(2)  # 設置接收超時時間
                response = s.recv(1024)
                print("接收到回應:", response.decode('utf-8'))
            except socket.timeout:
                print("未收到回應")
            
            return True
    except Exception as e:
        print("連線錯誤:", e)
        return False

def main():
    # 解析命令行參數
    parser = argparse.ArgumentParser(description='生成一組點位並傳送至ROS節點')
    parser.add_argument('--host', type=str, default='10.16.128.96', help='目標主機名或IP地址')
    parser.add_argument('--port', type=int, default=65432, help='目標端口')
    parser.add_argument('--pattern', type=str, default='random', choices=['random', 'circle', 'square', 'line'], 
                        help='點位模式: random(隨機), circle(圓形), square(正方形), line(直線)')
    parser.add_argument('--count', type=int, default=5, help='要生成的點位數量')
    parser.add_argument('--radius', type=float, default=0.2, help='圓形模式的半徑')
    parser.add_argument('--side', type=float, default=0.2, help='正方形模式的邊長')
    parser.add_argument('--height', type=float, default=0.2, help='點位的高度')
    parser.add_argument('--min', type=float, default=-0.25, help='隨機模式的座標最小值')
    parser.add_argument('--max', type=float, default=0.25, help='隨機模式的座標最大值')
    parser.add_argument('--start', type=float, nargs=3, default=[0.5, 0.5, 0.0], help='直線模式的起點坐標 [x y z]')
    parser.add_argument('--end', type=float, nargs=3, default=[0.5, 0.5, 1.0], help='直線模式的終點坐標 [x y z]')
    
    args = parser.parse_args()
    
    # 根據選擇的模式生成點位
    if args.pattern == 'random':
        points = generate_random_points(args.count, args.min, args.max)
        print(f"已生成 {len(points)} 個隨機點位")
    elif args.pattern == 'circle':
        points = generate_circle_points(args.count, args.radius, args.height)
        print(f"已生成 {len(points)} 個圓形軌跡點位，半徑: {args.radius}")
    elif args.pattern == 'square':
        points_per_side = max(2, args.count // 4)  # 確保每邊至少有2個點
        points = generate_square_points(args.side, args.height, points_per_side)
        print(f"已生成 {len(points)} 個正方形軌跡點位，邊長: {args.side}")
    elif args.pattern == 'line':
        points = generate_line_points(args.start, args.end, args.count)
        print(f"已生成 {len(points)} 個直線軌跡點位，從 {args.start} 到 {args.end}")
    
    # 顯示生成的點位
    for i, point in enumerate(points):
        print(f"點位 {i+1}/{len(points)}: x={point['x']}, y={point['y']}, z={point['z']}")
    
    # 傳送點位
    print(f"將傳送點位至 {args.host}:{args.port}")
    if not send_points(points, args.host, args.port):
        print("傳送失敗，中止操作")

if __name__ == '__main__':
    main()