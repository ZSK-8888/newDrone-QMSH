# Mavros+px4+gazabo模拟不完全指南

**张书恺 zsk200504@0utlook.com**

## 模拟原理

https://docs.px4.io/main/en/simulation/ros_interface

## 环境配置 

系统版本Ubuntu 20.04（与机载计算机同版本），不建议使用WSL/macOS，据传可能有兼容性问题

ROS配置：建议使用鱼香ros一键安装ros1-noetic

```shell
wget http://fishros.com/install -O fishros && . fishros
```

Mavros安装：参考https://docs.px4.io/v1.14/en/ros/mavros_installation.html

PX4安装+模拟

```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash /Tools/setup/ubuntu.sh
# 接下来重启机器后
sudo make px4_sitl gazebo-classic

# 如果报包缺失，可以尝试
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
```

## offboard控制程序样例和编译

在此之前，最好掌握一些ros基础，了解工作空间、包、节点的概念和一些基本操作

**例：**创建工作空间zskws，创建包simulation1

```shell
mkdir /zskws/src
cd zskws/src
catkin_init_workspace
# 创建一个新的 ROS 包
catkin_create_pkg simulation1 roscpp mavros_msgs rospython
```

将起飞样例`takeoff.cpp`放在`zskws/src/simulation1/src/takeoff.cpp`下

`takeoff2.cpp`是一个起飞，飞跃几个目标点再降落的样例，也可以尝试

编辑`CMakeList.txt`

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
)

# 包含 MAVROS 消息头文件
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp mavros_msgs
  DEPENDS
)

# 添加可执行文件
# 格式: add_executable(可执行文件名 C++源文件名)
add_executable(takeoff_node src/takeoff.cpp)

# 链接可执行文件所需的库
target_link_libraries(offboard_control_node
  ${catkin_LIBRARIES}
)
```



切换到工作空间根目录，编译

```shell
cd zskws
catkin_make
```





## 启动

开三个terminal，都要source ros环境

启动px4模拟器

```shell
source /opt/ros/noetic/setup.bash
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

模拟器参数参考[构建 PX4 软件 |PX4 指南（主要） --- Building PX4 Software | PX4 Guide (main)](https://docs.px4.io/main/en/dev_setup/building_px4#px4-make-build-targets)

启动mavros节点

```shell
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

启动offboard控制节点

```shell
source /opt/ros/noetic/setup.bash
# Source 你的工作空间
source ~/<你的工作空间名>/devel/setup.bash
rosrun simulation1 takeoff_node
```



