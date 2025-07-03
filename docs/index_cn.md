# ROS桥接文档

这是ROS bridge的文档，它实现了ROS和CARLA之间的双向通信。来自CARLA服务器的信息被转换为ROS主题。同样，ROS节点之间发送的消息也被转换为要在CARLA中应用的命令。

ROS bridge兼容ROS 1和ROS 2。

ROS bridge具有以下功能：

- 提供LIDAR、语义LIDAR、摄像头(深度、分割、RGB、DVS)、GNSS、雷达和IMU的传感器数据
- 提供对象数据，如变换、交通灯状态、可视化标记、碰撞和车道入侵
- 通过转向、油门和刹车控制AD代理
- 控制CARLA模拟的各个方面，如同步模式、播放和暂停模拟以及设置模拟参数

---

## 开始使用

- [__为ROS 1安装ROS bridge__](ros_installation_ros1_cn.md)
- [__为ROS 2安装ROS bridge__](ros_installation_ros2_cn.md)

---

## 了解主要的ROS bridge包

- [__CARLA ROS bridge__](run_ros_cn.md) - 运行ROS桥接所需的主要包
- [__ROS兼容性节点__](ros_compatibility_cn.md) - 允许相同API调用ROS 1或ROS 2函数的接口

---

## 了解其他ROS bridge包

- [__CARLA生成对象__](carla_spawn_objects_cn.md) - 提供生成actor的通用方法
- [__CARLA手动控制__](carla_manual_control.md) - 用于ego车辆的基于ROS的可视化和控制工具(类似于CARLA提供的`carla_manual_control.py`)
- [__CARLA Ackerman控制__](carla_ackermann_control.md) - 将ackermann命令转换为转向/油门/刹车的控制器
- [__CARLA路径点发布器__](carla_waypoint.md) - 发布和查询CARLA路径点
- [__CARLA AD代理__](carla_ad_agent.md) - 一个遵循路线、避免碰撞并遵守交通灯的示例代理
- [__CARLA AD演示__](carla_ad_demo.md) - 提供启动带有AD车辆的CARLA ROS环境所需的所有内容的示例包
- [__CARLA ROS场景运行器__](carla_ros_scenario_runner.md) - 通过ROS执行带有CARLA场景运行器的OpenScenarios的包装器
- [__CARLA Twist到控制__](carla_twist_to_control.md) - 将twist控制转换为CARLA车辆控制
- [__RVIZ插件__](rviz_plugin.md) - 可视化/控制CARLA的RVIZ插件
- [__RQT插件__](rqt_plugin.md) - 控制CARLA的RQT插件
- [__PCL记录器__](pcl_recorder.md) - 从模拟捕获的数据创建点云地图

---

## 探索参考资料

- [__ROS传感器__](ros_sensors.md) - 不同传感器中可用的参考主题
- [__ROS消息__](ros_msgs.md) - CARLA ROS消息中可用的参考参数
