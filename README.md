# WPR系列机器人仿真工具

## 准备工作

1. 安装ROS桌面完整版(kinetic/indigo):
```
sudo apt-get install ros-indigo-desktop-full
```
```
sudo apt-get install ros-kinetic-desktop-full
```
2. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/wpr_simulation.git
```
3. 编译
```
cd ~/catkin_ws
catkin_make
```

## 使用说明

### 1. 启智ROS机器人
简单场景:
```
roslaunch wpr_simulation wpb_simple.launch
```
![wpb_simple pic](./media/wpb_simple.png)
