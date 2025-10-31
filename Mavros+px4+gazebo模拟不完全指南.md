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

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash /Tools/setup/ubuntu.sh
# 接下来重启机器
make px4_sitl gazebo-classic

# 如果报包缺失，可以尝试
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
```



# 启动

source ros环境

```shell
source /opt/ros/noetic/setup.bash
```

启动px4模拟器

make px4_sitl gazebo-classic

参数参考

[构建 PX4 软件 |PX4 指南（主要） --- Building PX4 Software | PX4 Guide (main)](https://docs.px4.io/main/en/dev_setup/building_px4#px4-make-build-targets)
