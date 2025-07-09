# ROS/ROS2 bridge for CARLA simulator

[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)

 This ROS package is a bridge that enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

![rviz setup](./docs/images/ad_demo.png "AD Demo")

**This version requires CARLA 0.9.15**

## Features

- Provide Sensor Data (Lidar, Semantic lidar, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Play/pause simulation, Set simulation parameters)

## Getting started and documentation

Installation instructions and further documentation of the ROS bridge and additional packages are found [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/).

# Install ROS 
一键安装指令(ROS Noetic — For Ubuntu 20.04)
```
wget http://fishros.com/install -O fishros && . fishros 
```
# Install Carla 0.9.15
参考: https://carla.readthedocs.io/en/0.9.15/

# ROS bridge installation Using the source repository

1. Create a catkin workspace:
```
mkdir -p ~/carla-ros-bridge/catkin_ws/src
```
2. Clone the ROS Bridge repository and submodules:
```
cd ~/carla-ros-bridge
git clone --recurse-submodules -b 0.9.15 git@github.com:lichengguang/ros-bridge.git catkin_ws/src/ros-bridge
```
3. Set up the ROS environment according to the ROS version you have installed:
```
source /opt/ros/noetic/setup.bash
```
4. Install the required ros-dependencies:
```
sudo apt-get install ros-noetic-derived-object-msgs
sudo apt-get install ros-noetic-ackermann-msgs

cd catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r
```

5. Build the ROS bridge:
```
catkin build 
```
# Run the ROS bridge

1. Start a CARLA server according to the installation method used to install CARLA:
```
# Package version in carla root folder
./CarlaUE4.sh

# Debian installation in `opt/carla-simulator/`
./CarlaUE4.sh

# Build from source version in carla root folder
make launch
```
you can use Low quality mode
```
./CarlaUE4.sh -quality-level=Low
```
2. Add the correct CARLA modules to your Python path:
```
export CARLA_ROOT=<path-to-carla>
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla

```
for me this is:
```
export CARLA_ROOT=/home/lcg/workspace/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla

```
you can set it into ~/.bashrc
```
sudo vim ~/.bashrc
# add CARLA_ROOT and PYTHONPATH

export CARLA_ROOT=/home/lcg/workspace/CARLA_0.9.15
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla

# save and source 
source ~/.bashrc
```

3. Add the source path for the ROS bridge workspace according to the installation method of the ROS bridge. This should be done in every terminal each time you want to run the ROS bridge:
```
source devel/setup.bash
```
4. Start the ROS bridge. Use any of the different launch files available to check the installation:
```
# Option 1: start the ros bridge
roslaunch carla_ros_bridge carla_ros_bridge.launch

# Option 2: start the ros bridge together with an example ego vehicle
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```

更多示例参考官网文档:https://carla.readthedocs.io/projects/ros-bridge/en/latest/
