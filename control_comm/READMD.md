>robot_control
這個節點用於外部輸入點座標(x,y,z)進入RVIZ進行模擬。
執行：
```
python control.py --pattern line --count 5 --start -0.5 0.8 0.5 --end 0.5 0.8 1.5    
```
這會將兩個座標等分成5個點輸入ROS端。

ROS端：
執行`robot_control`。

