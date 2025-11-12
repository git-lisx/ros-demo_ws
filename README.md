# ros-demo
ROS Demo

创建功能包
cd ~/lisx/ros-demo_ws/src
catkin_create_pkg xian_pkg std_msgs rospy roscpp


教程链接

[ROS教程](http://www.autolabor.com.cn/book/ROSTutorials/)

## Service服务

在标准服务（std_srvs）只有这 3 个最基础的服务定义，：

| 服务类型               | 请求字段        | 响应字段                           | 常见用途           |
| ------------------ | ----------- | ------------------------------ | -------------- |
| `std_srvs/Empty`   | 无           | 无                              | 只触发一个动作，不需要参数  |
| `std_srvs/SetBool` | `bool data` | `bool success, string message` | 控制开/关类操作       |
| `std_srvs/Trigger` | 无           | `bool success, string message` | 触发一次操作（返回是否成功） |
