# CARLA自动驾驶演示

[自动驾驶演示](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_demo)是一个示例包，提供了启动带有自动驾驶车辆的CARLA ROS环境所需的所有内容。

- [__准备工作__](#准备工作)
- [__运行演示__](#运行演示)
    - [随机路线](#随机路线)
    - [场景执行](#场景执行)

---

## 准备工作

安装[Scenario Runner](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/)并按照Scenario Runner["入门教程"](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md)验证其是否正常工作。

设置环境变量以找到Scenario Runner安装路径：

```sh
export SCENARIO_RUNNER_PATH=<path_to_scenario_runner>
```

---

## 运行演示

#### 随机路线

要启动一个ego车辆跟随随机生成路线的演示，在启动CARLA服务器后运行以下命令：

```sh
# ROS 1
roslaunch carla_ad_demo carla_ad_demo.launch

# ROS 2
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

您还可以在另一个终端中执行以下命令生成额外的车辆或行人：

```sh
cd <CARLA_PATH>/PythonAPI/examples/

python3 spawn_npc.py
```

#### 场景执行

要使用预定义场景执行演示，在启动CARLA服务器后运行以下命令：

```sh
# ROS1
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch

# ROS2
ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py
```

在RVIZ Carla插件中选择示例场景"FollowLeadingVehicle"并点击"Execute"。ego车辆将被重新定位，场景将被处理。

您可以通过发布到`/carla/available_scenarios`来指定自己的场景。[启动文件](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch)展示了如何执行此操作的示例：

```launch
  <node pkg="rostopic" type="rostopic" name="publish_scenarios"
    args="pub /carla/available_scenarios carla_ros_scenario_runner_types/CarlaScenarioList '{ 'scenarios':  
      [
        {
          'name': 'FollowLeadingVehicle',
          'scenario_file': '$(find carla_ad_demo)/config/FollowLeadingVehicle.xosc'
        }
      ]
    }' -l"/>
```

---
