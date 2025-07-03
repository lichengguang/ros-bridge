# ROS桥接包

`carla_ros_bridge`包是运行基本ROS桥接功能所需的主要包。本节将介绍如何准备ROS环境、运行ROS桥接、配置设置、使用同步模式、控制ego车辆以及可用的订阅、发布和服务的摘要。

- [__设置ROS环境__](#设置ros环境)
    - [准备ROS 1环境](#准备ros-1环境)
    - [准备ROS 2环境](#准备ros-2环境)
- [__运行ROS桥接__](#运行ros桥接)
- [__配置CARLA设置__](#配置carla设置)
- [__在同步模式下使用ROS桥接__](#在同步模式下使用ros桥接)
- [__Ego车辆控制__](#ego车辆控制)
- [__ROS API__](#ros-api)
    - [订阅](#订阅)
    - [发布](#发布)
    - [服务](#服务)

---

## 设置ROS环境

ROS桥接支持ROS 1和ROS 2，使用具有共同接口的单独实现。当您想运行ROS桥接时，必须在每个使用的终端中根据ROS版本设置ROS环境：

#### 准备ROS 1环境：

运行的命令取决于您是通过Debian包还是通过源代码构建安装的ROS桥接。对于Debian选项，您还需要更改路径中的ROS版本：

```sh
    # 对于ROS桥接的Debian安装。根据您安装的ROS版本更改命令。
    source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

    # 对于GitHub仓库安装的ROS桥接
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

#### 准备ROS 2环境：

```sh
    source ./install/setup.bash
```

## 运行ROS桥接

设置好ROS环境并运行CARLA服务器后，您需要先启动`carla_ros_bridge`包才能使用任何其他包。为此，运行以下命令：

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

还有其他启动文件结合了上述功能，在启动ROS桥接的同时启动其他包或插件：

- `carla_ros_bridge_with_example_ego_vehicle.launch` (ROS 1) 和 `carla_ros_bridge_with_example_ego_vehicle.launch.py` (ROS 2) 启动ROS桥接以及[`carla_spawn_objects`](carla_spawn_objects_cn.md)和[`carla_manual_control`](carla_manual_control.md)包。

---

## 配置CARLA设置

配置应在启动文件中设置或在从命令行运行文件时作为参数传递，例如：

```sh
roslaunch carla_ros_bridge carla_ros_bridge.launch passive:=True
```

以下设置可用：

* __use_sim_time__: 应设置为__True__以确保ROS使用模拟时间而不是系统时间。此参数将使ROS [`/clock`][ros_clock]主题与CARLA模拟时间同步。
*  __host和port__: 使用Python客户端连接到CARLA的网络设置。
* __timeout__: 等待成功连接到服务器的时间。
* __passive__: 被动模式用于同步模式。启用后，ROS桥接将退居二线，另一个客户端__必须__tick世界。ROS桥接将等待接收来自所有传感器的所有预期数据。
*  __synchronous_mode__:
    *  __如果为false__: 数据在每个`world.on_tick()`和每个`sensor.listen()`回调上发布。
    *  __如果为true(默认)__: ROS桥接在下一个tick之前等待所有预期的传感器消息。这可能会减慢整体模拟速度，但确保结果可重现。
*  __synchronous_mode_wait_for_vehicle_control_command__: 在同步模式下，暂停tick直到完成车辆控制。
*  __fixed_delta_seconds__: 模拟步骤之间的模拟时间(增量秒)。__必须小于0.1__。查看[文档](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)了解更多信息。
*  __ego_vehicle__: 用于识别ego车辆的角色名称。将创建相关主题，以便这些车辆可以从ROS控制。
* __town__: 使用可用的CARLA城镇(例如'town01')或OpenDRIVE文件(以`.xodr`结尾)。
*  __register_all_sensors__:
    *  __如果为false__: 仅注册由桥接生成的传感器。
    *  __如果为true(默认)__: 注册模拟中存在的所有传感器。

[ros_clock]: https://wiki.ros.org/Clock

---

## 在同步模式下使用ROS桥接

ROS桥接默认在同步模式下运行。它将等待当前帧中预期的所有传感器数据，以确保结果可重现。

当在同步模式下运行多个客户端时，只允许一个客户端tick世界。除非启用被动模式，否则ROS桥接默认将是唯一允许tick世界的客户端。在[`ros-bridge/carla_ros_bridge/config/settings.yaml`](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/config/settings.yaml)中启用被动模式将使ROS桥接后退并允许另一个客户端tick世界。__另一个客户端必须tick世界，否则CARLA将冻结。__

如果ROS桥接不在被动模式下(ROS桥接是tick世界的那个)，那么有两种方法可以向服务器发送步骤控制：

- 向主题`/carla/control`发送带有[`carla_msgs.CarlaControl`](ros_msgs.md#carlacontrolmsg)消息的消息。
- 使用[Control rqt插件](rqt_plugin.md)。此插件启动一个带有简单界面的新窗口。然后用于管理步骤并发布到`/carla/control`主题。要使用它，在同步模式下运行以下命令：
```sh
    rqt --standalone rqt_carla_control
```

---

## Ego车辆控制

有两种模式可以控制ego车辆：

1. 正常模式 - 从`/carla/<ROLE NAME>/vehicle_control_cmd`读取命令
2. 手动模式 - 从`/carla/<ROLE NAME>/vehicle_control_cmd_manual`读取命令。这允许手动覆盖由软件堆栈发布的车辆控制命令。

您可以通过发布到`/carla/<ROLE NAME>/vehicle_control_manual_override`在两种模式之间切换。有关使用示例，请参见[Carla Manual Control](carla_manual_control.md)。

从命令行测试转向：

__1.__ 启动带有ego车辆的ROS桥接：

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

__2.__ 在另一个终端中，发布到主题`/carla/<ROLE NAME>/vehicle_control_cmd`

```sh
    # 最大前进油门和最大向右转向

    # ros1
    rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

    # ros2
    ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

```

车辆的当前状态可以通过主题`/carla/<ROLE NAME>/vehicle_status`接收。车辆的静态信息可以通过`/carla/<ROLE NAME>/vehicle_info`接收。

可以使用[AckermannDrive](https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html)消息来控制ego车辆。这可以通过使用[CARLA Ackermann Control](carla_ackermann_control.md)包来实现。

---

## ROS API

#### 订阅

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/debug_marker` | [visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html) | 在CARLA世界中绘制标记。 |
| `/carla/weather_control` | [carla_msgs/CarlaWeatherParameters](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaWeatherParameters.msg) | 设置CARLA天气参数 |
| `/clock` | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | 在ROS中发布模拟时间。 |

<br>

!!! 注意
    使用`debug_marker`时，请注意标记可能会影响传感器发布的数据。支持的标记包括：箭头(由两点指定)、点、立方体和线条带。
<br>

#### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/status` | [carla_msgs/CarlaStatus](ros_msgs.md#carlastatusmsg) | 读取CARLA的当前状态 |
| `/carla/world_info` | [carla_msgs/CarlaWorldInfo](ros_msgs.md#carlaworldinfomsg) | 关于当前CARLA地图的信息。 |
| `/clock` | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | 在ROS中发布模拟时间。 |
| `/rosout` | [rosgraph_msgs/Log](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html) | ROS日志记录。 |

<br>

#### 服务

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/destroy_object` | [carla_msgs/DestroyObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/DestroyObject.srv) | 销毁对象 |
| `/carla/get_blueprints` | [carla_msgs/GetBlueprints.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/GetBlueprints.srv) | 获取蓝图 |
| `/carla/spawn_object` | [carla_msgs/SpawnObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/SpawnObject.srv) | 生成对象 |

---
