#include "ros/ros.h"
#include "xian_pkg/AddTwoInts.h" // 替换成你的包名

// 回调函数：接收请求并返回响应
bool handle_add(xian_pkg::AddTwoInts::Request &req,
                xian_pkg::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("收到请求: a = %ld, b = %ld", (long)req.a, (long)req.b);
    ROS_INFO("返回结果: sum = %ld", (long)res.sum);
    return true;
}

int main(int argc, char **argv)
{
    // 设置中文环境
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("add_two_ints", handle_add);
    ROS_INFO("服务节点已启动，等待客户端请求...");
    ros::spin();

    return 0;
}
