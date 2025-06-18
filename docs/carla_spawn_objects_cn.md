# Carla生成对象

[`carla_spawn_objects`包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_spawn_objects)用于生成actor(车辆、传感器、行人)并将传感器附加到它们上。

- [__配置和传感器设置__](#配置和传感器设置)
    - [创建配置](#创建配置)
- [__生成车辆__](#生成车辆)
    - [重新生成车辆](#重新生成车辆)
- [__生成传感器__](#生成传感器)
    - [将传感器附加到现有车辆](#将传感器附加到现有车辆)

---

## 配置和传感器设置

对象及其附加的传感器通过`.json`文件定义。该文件的默认位置在`carla_spawn_objects/config/objects.json`。要更改位置，在启动包时通过私有ROS参数`objects_definition_file`传递文件路径：

```sh
    # ROS 1
    roslaunch carla_spawn_objects carla_spawn_objects.launch objects_definition_file:=path/to/objects.json

    # ROS 2
    ros2 launch carla_spawn_objects carla_spawn_objects.launch.py objects_definition_file:=path/to/objects.json
```

### 创建配置

您可以在[ros-bridge仓库][objectsjson]中找到示例，并按照以下大纲创建自己的配置和传感器设置：

```json
{
"objects":
    [
        {
            "type": "<SENSOR-TYPE>",
            "id": "<NAME>",
            "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            <ADDITIONAL-SENSOR-ATTRIBUTES>
        },
        {
            "type": "<VEHICLE-TYPE>",
            "id": "<VEHICLE-NAME>",
            "spawn_point": {"x": -11.1, "y": 138.5, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": -178.7},
            "sensors":
                [
                <SENSORS-TO-ATTACH-TO-VEHICLE>
                ]
        }
        ...
    ]
}
```

!!! 注意
    直接定义位置时请记住ROS使用[右手坐标系](https://www.ros.org/reps/rep-0103.html#chirality)

所有传感器属性都按照[蓝图库](https://carla.readthedocs.io/en/latest/bp_library/)中的描述定义。

[objectsjson]: https://github.com/carla-simulator/ros-bridge/blob/master/carla_spawn_objects/config/objects.json

---

## 生成车辆

- 如果没有定义特定的生成点，车辆将在随机位置生成。
- 要定义车辆生成的位置，有两种选择：

    - 将所需位置传递给ROS参数`spawn_point_<VEHICLE-NAME>`。`<VEHICLE-NAME>`是您在`.json`文件中给车辆的`id`：

            # ROS 1
            roslaunch carla_spawn_objects carla_spawn_objects.launch spawn_point_<VEHICLE-NAME>:=x,y,z,roll,pitch,yaw

            # ROS 2
            ros2 launch carla_spawn_objects carla_spawn_objects.launch.py spawn_point_<VEHICLE-NAME>:=x,y,z,roll,pitch,yaw

    - 直接在`.json`文件中定义初始位置：

            {
            "type": "vehicle.*",
            "id": "ego_vehicle",
            "spawn_point": {"x": -11.1, "y": 138.5, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": -178.7},
            }

### 重新生成车辆

在模拟过程中，可以通过向主题`/carla/<ROLE NAME>/<CONTROLLER_ID>/initialpose`发布消息将车辆重新生成到不同位置。要使用此功能：

1. 在`.json`文件中将`actor.pseudo.control`伪actor附加到车辆上。它应该具有与用于发布到主题的`<CONTROLLER_ID>`值相同的`id`值：

        {
        "type": "vehicle.*",
        "id": "ego_vehicle",
        "sensors":
        [
            {
            "type": "actor.pseudo.control",
            "id": "control"
            }
        ]
        }

2. 启动`set_inital_pose`节点，将`<CONTROLLER_ID>`作为参数传递给ROS参数`controller_id`(默认为'control')：

        roslaunch carla_spawn_objects set_initial_pose.launch controller_id:=<CONTROLLER_ID>

3. 设置新位置的推荐方法是使用RVIZ界面中的__2D Pose Estimate__按钮。然后您可以点击地图视口在该位置重新生成。这将删除当前的`ego_vehicle`并在指定位置重新生成它。

> ![rviz_set_start_goal](images/rviz_set_start_goal.png)

---

## 生成传感器

- 传感器的初始位置应直接在`.json`文件中定义，如上面车辆所示。
- 附加到车辆的传感器的生成点被认为是相对于车辆的。

### 将传感器附加到现有车辆

传感器可以附加到已经存在的车辆上。为此：

1. 在`.json`文件中定义伪传感器`sensor.pseudo.actor_list`。这将提供对已存在actor列表的访问。

        ...
        {
            "type": "sensor.pseudo.actor_list",
            "id": "actor_list"
        },

2. 根据需要定义其余传感器。
3. 启动节点时将`spawn_sensors_only`参数设置为True。这将检查是否已存在与`.json`文件中指定的相同`id`和`type`的actor，如果存在，则将传感器附加到此actor。

        # ROS 1
        roslaunch carla_spawn_objects carla_spawn_objects.launch spawn_sensors_only:=True

        # ROS 2
        ros2 launch carla_spawn_objects carla_spawn_objects.launch.py spawn_sensors_only:=True

---
