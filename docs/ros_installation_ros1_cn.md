# ROS 1桥接安装指南

本节介绍如何在Linux上安装ROS桥接以用于ROS 1。您将找到先决条件、安装步骤、如何运行基本包以确保一切正常工作以及运行测试的命令。

- [__开始之前__](#开始之前)
    - [__ROS桥接安装__](#ros桥接安装)
        - [A. 使用Debian仓库](#a-使用debian仓库)
        - [B. 使用源代码仓库](#b-使用源代码仓库)
- [__运行ROS桥接__](#运行ros桥接)
- [__测试__](#测试)

!!! 重要
    ROS桥接尚未在Windows上进行测试。

---
## 开始之前

在使用ROS桥接之前，您需要满足以下软件要求：

- 根据您的操作系统安装ROS：
    - [__ROS Melodic__](https://wiki.ros.org/melodic/Installation/Ubuntu) — 适用于Ubuntu 18.04 (Bionic)
    - [__ROS Noetic__](https://wiki.ros.org/noetic#Installation) — 适用于Ubuntu 20.04 (Focal)
- 根据您的需求可能需要额外的ROS包。强烈推荐使用[rviz](https://wiki.ros.org/rviz)来可视化ROS数据。
- CARLA 0.9.7或更高版本 — 早期版本与ROS桥接不兼容。按照[快速启动安装](https://carla.readthedocs.io/en/latest/start_quickstart/)或为[Linux](https://carla.readthedocs.io/en/latest/build_linux/)进行构建。建议尽可能使ROS桥接版本与CARLA版本匹配。

---
## ROS桥接安装

有两种安装ROS桥接的选项：

- 通过__apt__工具从Debian仓库安装。仅在Ubuntu 18.04上可用。
- 从GitHub上的源代码仓库克隆。

两种方法详细说明如下。

!!! 重要
    要安装0.9.10之前的ROS桥接版本，您可以在CARLA文档的旧版本中找到说明[此处](https://carla.readthedocs.io/en/0.9.10/ros_installation/)。使用窗口右下角的面板切换到适当版本的文档。![docs_version_panel](images/docs_version_panel.jpg)

### A. 使用Debian仓库

!!! 注意
    此安装方法仅在Ubuntu 18.04上可用。对于其他支持的发行版，请参见[B部分：使用源代码仓库](#b-使用源代码仓库)。

__1.__ 在系统中设置Debian仓库：
```sh
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
```

__2.__ 安装ROS桥接：

> - 最新版本：
```sh
        sudo apt-get update # 更新Debian包索引
        sudo apt-get install carla-ros-bridge # 安装最新ROS桥接版本，或更新当前安装
```

> - 通过添加版本标签安装特定版本：
```sh
        apt-cache madison carla-ros-bridge # 列出可用的ROS桥接版本
        sudo apt-get install carla-ros-bridge=0.9.10-1 # 本例中，"0.9.10"指ROS桥接版本，"1"指Debian修订版本
```

__3.__ 检查ROS桥接是否已成功安装在`/opt/`文件夹中。

### B. 使用源代码仓库

__1.__ 创建catkin工作空间：
```sh
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
```

__2.__ 克隆ROS桥接仓库和子模块：
```sh
    cd ~/carla-ros-bridge
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git catkin_ws/src/ros-bridge
```

__5.__ 根据您安装的ROS版本设置ROS环境：
```sh
    source /opt/ros/<melodic/noetic>/setup.bash
```
__6.__ 安装所需的ros依赖项：
```sh
    cd catkin_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r
```

__7.__ 构建ROS桥接：
```sh
    catkin build   # 也可以使用catkin_make
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

__3.__ 根据ROS桥接的安装方法添加ROS桥接工作空间的源路径。每次运行ROS桥接时，都应在每个终端中执行此操作：

```sh
    # 对于ROS桥接的Debian安装。根据您安装的ROS版本更改命令。
    source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

    # 对于GitHub仓库安装的ROS桥接
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

!!! 重要
    可以永久设置源路径，但在使用其他工作空间时可能会导致冲突。

__4.__ 启动ROS桥接。使用任何可用的启动文件来检查安装：

```sh
    # 选项1: 启动ros桥接
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # 选项2: 启动ros桥接和一个示例ego车辆
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
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

使用catkin执行测试：

__1.__ 构建包：

```sh
    catkin_make -DCATKIN_ENABLE_TESTING=0
```

__2.__ 运行测试：

```sh
    rostest carla_ros_bridge ros_bridge_client.test
```

---
