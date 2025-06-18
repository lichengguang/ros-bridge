# CARLA Ackermann控制

[`carla_ackermann_control`包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ackermann_control)用于通过[Ackermann消息][ackermanncontrolmsg]控制CARLA车辆。该包将Ackermann消息转换为[CarlaEgoVehicleControl][carlaegovehiclecontrolmsg]消息。它从CARLA读取车辆信息，并将这些信息传递给基于Python的PID控制器`simple-pid`来控制加速度和速度。

[ackermanncontrolmsg]: https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html
[carlaegovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#carlaegovehiclecontrolmsg

- [__配置__](#配置)
- [__测试控制消息__](#测试控制消息)
- [__ROS API__](#ros-api)
    - [订阅](#订阅)
    - [发布](#发布)

---

### 配置

参数可以在使用ROS 1和ROS 2时通过[配置文件][ackermanconfig]初始设置，在ROS 1中也可以通过ROS[动态重配置][rosdynamicreconfig]在运行时设置。

[ackermanconfig]: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/config/settings.yaml
[rosdynamicreconfig]: https://wiki.ros.org/dynamic_reconfigure

---

### 测试控制消息

通过向主题`/carla/<ROLE NAME>/ackermann_cmd`发送命令来测试设置。例如，以10米/秒的速度向前移动角色名为`ego_vehicle`的ego车辆，运行以下命令：

```bash
# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10
```

或者让车辆以1.22弧度的转向角前进：

```bash
# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10
```

---

### ROS API

#### 订阅

|主题|类型|描述|
|--|--|--|
|`/carla/<ROLE NAME>/ackermann_cmd` | [ackermann_msgs.AckermannDrive][ackermanncontrolmsg] | 转向命令的__订阅者__ |

<br>

#### 发布

|主题|类型|描述|
|--|--|--|
| `/carla/<ROLE NAME>/ackermann_control/control_info` | [carla_ackermann_control.EgoVehicleControlInfo][egovehiclecontrolmsg] | 控制器中使用的当前值(用于调试) |

[egovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#egovehiclecontrolinfomsg

<br>
