# ROS兼容性节点

[ROS兼容性节点](https://github.com/carla-simulator/ros-bridge/tree/master/ros_compatibility)是一个接口，允许包同时无缝地与ROS 1和ROS 2一起使用。根据环境变量`ROS_VERSION`，相同的API将调用ROS 1或ROS 2函数。通过创建继承自`CompatibleNode`的类来使用它。

---

## ROS参数

在ROS 2中，参数需要在设置或访问之前声明。而在ROS 1中则不需要。为了使ROS 1和ROS 2模式以类似的方式工作，ROS 2版本的`CompatibleNode`中将参数`allow_undeclared_parameters`设置为`True`，从而允许使用未事先声明的参数。

---

## 服务

在ROS 2中，服务可以异步调用。而在ROS 1中则不行。因此，ROS 2版本的`call_service()`方法在异步调用服务后会等待服务器的响应，以模拟ROS 1的同步行为。

!!! 警告
    在等待响应时，ROS 2的`call_service()`方法会旋转(spin)节点。如果另一个线程并行旋转同一个节点，可能会导致问题(错误或死锁)。
