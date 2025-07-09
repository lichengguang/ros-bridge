# CARLA 模拟器 ROS/ROS2 桥接器

[![文档状态](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)

本ROS包是一个桥接器，实现了ROS与CARLA模拟器之间的双向通信。CARLA服务器的信息会被转换为ROS话题，同时ROS节点间的消息也会被转换为CARLA可执行的命令。

![rviz演示](./docs/images/ad_demo.png "自动驾驶演示")

**当前版本要求 CARLA 0.9.15**

## 主要功能

- **传感器数据支持**：
  - LiDAR
  - 语义LiDAR
  - 摄像头（深度、语义分割、RGB、DVS）
  - GNSS
  - 雷达
  - IMU

- **对象数据支持**：
  - 变换（通过[tf](http://wiki.ros.org/tf)）
  - 交通灯状态
  - 可视化标记
  - 碰撞检测
  - 车道入侵检测

- **控制功能**：
  - 自动驾驶代理控制（转向/油门/刹车）
  - CARLA仿真控制（暂停/继续仿真，设置仿真参数）

## 系统要求

- **操作系统**：Ubuntu 20.04
- **ROS版本**：Noetic
- **CARLA版本**：0.9.15

## 安装指南

### 1. 安装ROS Noetic（Ubuntu 20.04）

使用一键安装指令：
```bash
wget http://fishros.com/install -O fishros && . fishros
```

### 2. 安装CARLA 0.9.15

参考官方文档：[CARLA 0.9.15 安装指南](https://carla.readthedocs.io/en/0.9.15/)

### 3. 安装ROS桥接器（源码方式）

1. 创建工作空间：
```bash
mkdir -p ~/carla-ros-bridge/catkin_ws/src
```

2. 克隆ROS桥接器仓库及子模块：
```bash
cd ~/carla-ros-bridge
git clone --recurse-submodules -b 0.9.15 git@github.com:lichengguang/ros-bridge.git catkin_ws/src/ros-bridge
```

3. 设置ROS环境：
```bash
source /opt/ros/noetic/setup.bash
```

4. 安装ROS依赖：
```bash
sudo apt-get install ros-noetic-derived-object-msgs
sudo apt-get install ros-noetic-ackermann-msgs

cd catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r
```

5. 构建ROS桥接器：
```bash
catkin build
```

## 使用说明

### 1. 启动CARLA服务器

根据您的安装方式选择相应命令：
```bash
# 源码包版本或Debian安装版本（在carla根目录）
./CarlaUE4.sh

# 源码编译版本（在carla根目录）
make launch
```

可以使用低质量模式节省资源：
```bash
./CarlaUE4.sh -quality-level=Low
```

### 2. 设置Python路径

```bash
export CARLA_ROOT=<path-to-carla>
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
```

示例（根据您的实际路径调整）：
```bash
export CARLA_ROOT=/home/lcg/workspace/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
```

可以将这些设置添加到`~/.bashrc`文件中永久生效：
```bash
echo 'export CARLA_ROOT=/home/lcg/workspace/CARLA_0.9.15' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla' >> ~/.bashrc
source ~/.bashrc
```

### 3. 启动ROS桥接器

首先设置工作空间环境：
```bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

启动选项：
```bash
# 选项1：仅启动ROS桥接器
roslaunch carla_ros_bridge carla_ros_bridge.launch

# 选项2：启动ROS桥接器并附带示例车辆
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```

## 示例与演示

更多示例和详细用法请参考官方文档：
[ROS桥接器官方文档](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)


## 功能包介绍

本项目包含以下主要功能包：

- **carla_ros_bridge**: 核心桥接功能，实现ROS与CARLA的基础通信
- **[carla_ackermann_control](./docs/carla_ackermann_control_cn.md)**: Ackermann车辆控制接口
- **[carla_ad_agent](./docs/carla_ad_agent_cn.md)**: 自动驾驶代理实现
- **[carla_manual_control](./docs/carla_manual_control_cn.md)**: 手动控制接口
- **[carla_ros_scenario_runner](./docs/carla_ros_scenario_runner_cn.md)**: 场景运行器集成
- **[carla_spawn_objects](./docs/carla_spawn_objects_cn.md)**: 对象生成工具
- **[carla_twist_to_control](./docs/carla_twist_to_control_cn.md)**: Twist消息转换控制
- **[carla_waypoint_publisher](./docs/carla_waypoint_cn.md)**: 路径点发布工具
- **rqt_carla_control**: RQT插件控制界面
- **[rviz_carla_plugin](./docs/rviz_plugin_cn.md)**: RVIZ可视化插件

各功能包详细文档可在`docs`目录下查看：

- 中文文档：`docs/*_cn.md`
- 英文文档：`docs/*.md` (无_cn后缀)

## 文档链接

- [CARLA官方文档](https://carla.readthedocs.io)
- [ROS桥接器文档](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)
