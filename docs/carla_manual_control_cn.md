# CARLA手动控制

[CARLA手动控制包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control)是CARLA自带的[`manual_control.py`][manualcontrol]脚本的纯ROS版本。所有数据都通过ROS主题接收。

[manualcontrol]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/manual_control.py

- [__需求__](#需求)
- [__运行包__](#运行包)

---

## 需求

要使用`carla_manual_control`，需要为ego车辆附加一些特定传感器(关于如何为车辆附加传感器，请参见[CARLA生成对象](carla_spawn_objects_cn.md))：

- __显示图像__：需要角色名为`rgb_view`、分辨率为800x600的摄像头
- __显示当前位置__：需要角色名为`gnss`的GNSS传感器和角色名为`odometry`的里程计伪传感器
- __获取车道入侵通知__：需要角色名为`lane_invasion`的车道入侵传感器
- __获取碰撞通知__：需要角色名为`collision`的碰撞传感器

---

## 运行包

运行包的步骤：

__1.__ 确保CARLA正在运行。启动ROS桥接：

```sh
        # ROS 1
        roslaunch carla_ros_bridge carla_ros_bridge.launch

        # ROS 2
        ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

__2.__ 生成对象：

```sh
        # ROS 1
        roslaunch carla_spawn_objects carla_spawn_objects.launch

        # ROS 2
        ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
```

__3.__ 启动`carla_manual_control`节点：

```sh
        # ROS 1
        roslaunch carla_manual_control carla_manual_control.launch

        # ROS 2
        ros2 launch carla_manual_control carla_manual_control.launch.py
```

__4.__ 要手动控制车辆，按'B'键。按'H'键查看操作说明。

或者，上述所有命令可以合并为一个单独的启动文件，通过执行以下命令同时运行：

```sh
        # ROS 1
        roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

        # ROS 2
        ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```
---
