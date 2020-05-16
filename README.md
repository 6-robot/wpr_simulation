# WPR系列机器人仿真工具

## 准备工作

1. 安装ROS桌面完整版(Kinetic/Ubuntu 16.04):
```
sudo apt-get install ros-kinetic-desktop-full
```
由于Indigo/Ubuntu 14.04集成的Gazebo版本太过古老，所以无法进行支持，建议升级到Kinetic/Ubuntu 16.04。

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
![wpb_simple pic](https://user-images.githubusercontent.com/17635413/82132384-cc705000-9811-11ea-89ee-20a4fb938f80.png)

SLAM环境地图创建:
```
roslaunch wpr_simulation wpb_gmapping.launch
```
![wpb_gmapping pic](https://user-images.githubusercontent.com/17635413/82132405-fa559480-9811-11ea-9b32-bd096e844d2b.png)

Navigation导航:
```
roslaunch wpr_simulation wpb_navigation.launch
```
![wpb_navigation pic](https://user-images.githubusercontent.com/17635413/82132412-093c4700-9812-11ea-959a-f40f1be40cab.png)