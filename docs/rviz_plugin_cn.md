# RVIZ Carla插件

[RVIZ插件](https://github.com/carla-simulator/ros-bridge/tree/master/rviz_carla_plugin)提供了基于[RVIZ](https://wiki.ros.org/rviz) ROS包的可视化工具。

- [__使用RVIZ运行ROS桥接__](#使用rviz运行ros桥接)
- [__RVIZ插件功能__](#rviz插件功能)
- [__ROS API__](#ros-api)
    - [订阅](#订阅)
    - [发布](#发布)
    - [服务](#服务)

---

## 使用RVIZ运行ROS桥接

![ros_rviz](images/ros_rviz.png)

RVIZ插件期望一个名为`ego_vehicle`的ego车辆。要在CARLA服务器运行时查看ROS桥接与RVIZ的工作示例，请执行以下命令：

__1.__ 启动启用RVIZ的ROS桥接：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

__2.__ 启动RVIZ：

```sh
# ROS 1
rosrun rviz rviz

# ROS 2
ros2 run rviz2 rviz2
```

__3.__ 使用`carla_spawn_objects`包生成ego车辆：

```sh
# ROS 1
roslaunch carla_spawn_objects carla_spawn_objects.launch

# ROS 2
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
```

__4.__ 使用`carla_manual_control`包控制ego车辆(按`B`键启用手动转向)：

```sh
# ROS 1
roslaunch carla_manual_control carla_manual_control.launch

# ROS 2
ros2 launch carla_manual_control carla_manual_control.launch.py
```

---

## RVIZ插件功能

- __ego车辆状态可视化__ - 可视化车辆位置和控制
- __向其他节点提供RVIZ视角姿态__ - 通过将`actor.pseudo.control`附加到相机上，通过发布Pose消息在CARLA世界中移动相机
- __传感器可视化__ - 可视化RGB、LIDAR、深度、DVS和语义分割相机信息
- __执行场景__ - 使用[carla_ros_scenario_runner](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_scenario_runner)包触发场景
- __播放/暂停模拟__ - 如果在同步模式下启动，可以播放和暂停模拟
- __手动覆盖ego车辆控制__ - 使用[RVIZ可视化教程](https://github.com/ros-visualization/visualization_tutorials)中的drive-widget和将twist转换为车辆控制的[节点](https://github.com/carla-simulator/ros-bridge/blob/master/carla_twist_to_control)，通过鼠标控制车辆

---

## ROS API

#### 订阅

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/status` | [carla_msgs/CarlaStatus](ros_msgs.md#carlastatusmsg) | 读取CARLA当前状态 |
| `/carla/ego_vehicle/vehicle_status` | [carla_msgs/CarlaEgoVehicleStatus](ros_msgs.md#carlaegovehiclestatusmsg) | 显示ego车辆当前状态 |
| `/carla/ego_vehicle/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | 显示ego车辆当前姿态 |
| `/scenario_runner/status` | [carla_ros_scenario_runner_types/CarlaScenarioRunnerStatus](ros_msgs.md#carlascenariorunnerstatusmsg) | 可视化场景运行器状态 |
| `/carla/available_scenarios` | [carla_ros_scenario_runner_types/CarlaScenarioList](ros_msgs.md#carlascenariolistmsg) | 提供要执行的场景列表(在组合框中禁用)|

<br>

#### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/control` | [carla_msgs/CarlaControl](ros_msgs.md#carlacontrolmsg) | 播放/暂停/单步执行CARLA |
| `/carla/ego_vehicle/spectator_pose` | [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) | 发布RVIZ相机视图的当前姿态 |
| `/carla/ego_vehicle/vehicle_control_manual_override` | [std_msgs/Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html) | 启用/禁用车辆控制覆盖 |
| `/carla/ego_vehicle/twist` | [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | 通过鼠标创建的twist命令 |

<br>

#### 服务

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/scenario_runner/execute_scenario` | [carla_ros_scenario_runner_types/ExecuteScenario](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) | 执行选定的场景 |

<br>
