# ROS 2桥接安装指南

本节介绍如何在Linux上安装ROS桥接以用于ROS 2。您将找到先决条件、安装步骤、如何运行基本包以确保一切正常工作以及运行测试的命令。

- [__开始之前__](#开始之前)
- [__ROS桥接安装__](#ros桥接安装)
- [__运行ROS桥接__](#运行ros桥接)
- [__测试__](#测试)

!!! 重要
    ROS在Windows上仍处于[实验阶段](http://wiki.ros.org/noetic/Installation)。目前仅在Linux系统上测试过。

---

## 开始之前

在使用ROS桥接之前，您需要满足以下软件要求：

- 安装ROS：
    - [__ROS 2 Foxy__](https://docs.ros.org/en/foxy/Installation.html) — 适用于Ubuntu 20.04 (Focal)
- 根据您的需求可能需要额外的ROS包。强烈推荐使用[rviz](https://wiki.ros.org/rviz)来可视化ROS数据。
- CARLA 0.9.11或更高版本 — 早期版本与ROS桥接不兼容。按照[快速启动安装](https://carla.readthedocs.io/en/latest/start_quickstart/)或为[Linux](https://carla.readthedocs.io/en/latest/build_linux/)进行构建。建议尽可能使ROS桥接版本与CARLA版本匹配。

---

## ROS桥接安装

!!! 注意
    ROS 2目前尚未提供Debian包安装方式。

__1.__ 设置项目目录并克隆ROS桥接仓库和子模块：

```sh
    mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
```

__2.__ 设置ROS环境：

```sh
    source /opt/ros/foxy/setup.bash
```

__3.__ 安装ROS依赖项：

```sh
    rosdep update
    rosdep install --from-paths src --ignore-src -r
```

__4.__ 使用colcon构建ROS桥接工作空间：

```sh
    colcon build
```

---

## 运行ROS桥接

__1.__ 根据安装CARLA的方法启动CARLA服务器：

```sh
    # 在carla根文件夹中的包版本
    ./CarlaUE4.sh

    # 在`opt/carla-simulator/`中的Debian安装
    ./CarlaUE4.sh

    # 从源代码构建版本在carla根文件夹中
    make launch
```

__2.__ 将正确的CARLA模块添加到Python路径：

```sh
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
```

__3.__ 添加ROS桥接工作空间的源路径：

```sh
    source ./install/setup.bash
```

__4.__ 在另一个终端中，启动ROS 2桥接。您可以运行以下两个选项之一：

```sh
    # 选项1，启动基本ROS桥接包
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

    # 选项2，启动带有示例ego车辆的ROS桥接
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

!!! 注意

    如果收到错误：`ImportError: no module named CARLA`，则缺少CARLA Python API的路径。apt安装会自动设置路径，但其他安装可能会缺少。

    您需要将适当的`.egg`文件添加到Python路径中。您可以在`/PythonAPI/`或`/PythonAPI/dist/`中找到该文件，具体取决于CARLA安装。使用与您安装的Python版本对应的文件执行以下命令：

    `export PYTHONPATH=$PYTHONPATH:path/to/carla/PythonAPI/<your_egg_file>`

    建议通过将上述行添加到`.bashrc`文件中永久设置此变量。

    要检查是否可以正确导入CARLA库，运行以下命令并等待成功消息：

            python3 -c 'import carla;print("Success")' # python3

            或

            python -c 'import carla;print("Success")' # python2

---

## 测试

使用colcon执行测试：

__1.__ 构建包：

```sh
    colcon build --packages-up-to carla_ros_bridge
```

__2.__ 运行测试：

```sh
    launch_test carla_ros_bridge/test/ros_bridge_client_ros2_test.py
```

---
