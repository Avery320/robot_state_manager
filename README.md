# robot_state_manager

## Introduction
此專案分為機器人節點(robot_comm)與控制節點(control_comm)兩個部分。
機器人節點位於ROS系統運行，用來獲取機器人的相關數據。
控制節點於macos\windows系統下運行，用來處理從ROS系統下所讀取的相關數據，進行其他應用。

## Nodes
### ROS
- joint_states_publisher
- position_publisher
- 
### macos/windows
- joint_states_receiver
- position_recerver


## 對應節點
|system|joint_state|position|
|:-:|:-:|:-:|
|種類|輸出|輸出|
|robot|joint_states_publisher|position_publisher|
|control|joint_states_receiver|position_recerver|

## Clone repo

- 機器人端(linux)下載ros-dev分支：
  ```
  git clone -b ros-dev git@github.com:Avery320/robot_state_manager.git
  ```
- 控制端(macOS/Windows)下載ctrl-dev分支：
  ```
  git clone -b ctrl-dev git@github.com:Avery320/robot_state_manager.git
  ```