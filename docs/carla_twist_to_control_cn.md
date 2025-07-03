# CARLA Twist到控制转换

[`carla_twist_to_control`包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_twist_to_control)将[geometry_msgs.Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)转换为[carla_msgs.CarlaEgoVehicleControl](ros_msgs.md#carlaegovehiclecontrolmsg)。

---
## ROS API

### 订阅

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/vehicle_info` | [`carla_msgs.CarlaEgoVehicleInfo`](ros_msgs.md#carlaegovehicleinfomsg) | Ego车辆信息，用于接收最大转向角度 |
| `/carla/<ROLE NAME>/twist` | [`geometry_msgs.Twist`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | 要转换的Twist消息 |

<br>

### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [`carla_msgs.CarlaEgoVehicleControl`](ros_msgs.md#carlaegovehiclecontrolmsg) | 转换后的车辆控制命令 |

<br>
