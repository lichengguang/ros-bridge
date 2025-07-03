# CARLA路径点发布器

[CARLA路径点发布器](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher)使路径点计算可用于ROS上下文，并提供查询CARLA路径点的服务。要了解更多关于路径点的信息，请参阅CARLA[文档](https://carla.readthedocs.io/en/latest/core_map/#navigation-in-carla)。

- [__运行路径点发布器__](#运行路径点发布器)
    - [设置目标](#设置目标)
- [__使用路径点发布器__](#使用路径点发布器)
- [__ROS API__](#ros-api)
    - [发布](#发布)
    - [服务](#服务)

---

## 运行路径点发布器

在CARLA服务器运行的情况下，执行以下命令：

```sh
# ROS 1
roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch

# ROS 2
ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
```

### 设置目标

如果有可用的目标，将从主题`/carla/<ROLE NAME>/goal`读取，否则使用固定的生成点。

设置目标的推荐方法是点击RVIZ中的'2D Nav Goal'。

![rviz_set_goal](images/rviz_set_start_goal.png)

---

### 使用路径点发布器

[CARLA AD演示](carla_ad_demo.md)使用路径点发布器为[CARLA AD代理](carla_ad_agent.md)规划路线。有关使用示例，请参阅CARLA AD演示[启动文件](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch)。

---

## ROS API

#### 发布

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<ego vehicle name>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | 发布计算出的路线 |

<br>

#### 服务

| 服务 | 类型 | 描述 |
|-------|------|-------------|
| `/carla_waypoint_publisher/<ego vehicle name>/get_waypoint` | [carla_waypoint_types/GetWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetWaypoint.srv) | 获取特定位置的路径点 |
| `/carla_waypoint_publisher/<ego vehicle name>/get_actor_waypoint` | [carla_waypoint_types/GetActorWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetActorWaypoint.srv) | 获取actor ID的路径点 |

<br>
