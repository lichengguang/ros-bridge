# CARLA自动驾驶代理

[CARLA自动驾驶代理](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_agent)是一个能够跟随给定路线、避免与其他车辆碰撞并遵守交通灯状态的自动驾驶代理，通过访问真实数据实现。它被[CARLA AD演示](carla_ad_demo.md)用来展示如何使用ROS桥接。

- [__需求__](#需求)
- [__ROS API__](#ros-api)
    - [__AD代理节点__](#ad代理节点)
        - [参数](#参数)
        - [订阅](#订阅)
        - [发布](#发布)
    - [__本地规划器节点__](#本地规划器节点)
        - [参数](#参数)
        - [订阅](#订阅)
        - [发布](#发布)

CARLA自动驾驶代理内部使用一个独立的节点进行[本地规划](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_agent/src/carla_ad_agent/local_planner.py)。该节点已针对`vehicle.tesla.model3`进行了优化，因为它没有换挡延迟。

PID参数是通过[齐格勒-尼科尔斯方法](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)收集的。

---

## 需求

要使用`carla_ad_agent`，需要生成一组最小传感器(关于如何生成传感器，请参见[CARLA生成对象](carla_spawn_objects_cn.md))：

- 附加到车辆的里程计伪传感器(`sensor.pseudo.odom`)，角色名为`odometry`
- 附加到车辆的对象伪传感器(`sensor.pseudo.objects`)，角色名为`objects`
- 交通灯伪传感器(`sensor.pseudo.traffic_lights`)，角色名为`traffic_lights`

---

## ROS API

### AD代理节点

#### 参数

| 参数 | 类型 | 描述 |
|-----------|------|-------------|
| `role_name` | string (默认: `ego_vehicle`) | ego车辆的CARLA角色名 |
| `avoid_risk` | bool (默认: `true`) | 如果为True，避免与其他车辆碰撞并遵守交通灯 |

<br>

#### 订阅

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/target_speed` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | ego车辆的目标速度 |
| `/carla/<ROLE NAME>/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | ego车辆的里程计 |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs/CarlaEgoVehicleInfo](ros_msgs.md#carlaegovehicleinfomsg) | 识别ego车辆的CARLA actor ID |
| `/carla/<ROLE NAME>/objects` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | 其他actor的信息 |
| `/carla/traffic_lights/status` | [carla_msgs/CarlaTrafficLightStatusList](ros_msgs.md#carlatrafficlightstatuslistmsg) | 获取交通灯的当前状态 |
| `/carla/traffic_lights/info` | [carla_msgs/CarlaTrafficLightInfoList](ros_msgs.md#carlatrafficlightinfolistmsg) | 获取交通灯信息 |

<br>

#### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | 目标速度 |

<br>

### 本地规划器节点

#### 参数

| 参数 | 类型 | 描述 |
|-----------|------|-------------|
| `role_name` | string (默认: `ego_vehicle`) | ego车辆的CARLA角色名 |
| `control_time_step` | float (默认: `0.05`) | 控制循环速率 |
| `Kp_lateral` | float (默认 `0.9`) | 横向PID控制器的比例项 |
| `Ki_lateral` | float (默认 `0.0`) | 横向PID控制器的积分项 |
| `Kd_lateral` | float (默认 `0.0`) | 横向PID控制器的微分项 |
| `Kp_longitudinal` | float (默认 `0.206`) | 纵向PID控制器的比例项 |
| `Ki_longitudinal` | float (默认 `0.0206`) | 纵向PID控制器的积分项 |
| `Kd_longitudinal` | float (默认 `0.515`) | 纵向PID控制器的微分项 |

<br>

#### 订阅

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | 要跟随的路线 |
| `/carla/<ROLE NAME>/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | ego车辆的里程计 |
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | 目标速度 |

<br>

#### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ROLE NAME>/next_target` | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | 下一个目标姿态标记 |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs/CarlaEgoVehicleControl](ros_msgs.md#carlaegovehiclecontrolmsg) | 车辆控制命令 |

<br>
