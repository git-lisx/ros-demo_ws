#include "ros/ros.h"
#include "xian_pkg/AddTwoInts.h" // 替换成你的包名
#include <cstdlib>

int main(int argc, char **argv)
{
    // 设置中文环境
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "add_two_ints_client");

    // if (argc != 3)
    // {
    //     ROS_INFO("用法: rosrun xian_pkg add_two_ints_client X Y");
    //     return 1;
    // }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<xian_pkg::AddTwoInts>("add_two_ints");

    xian_pkg::AddTwoInts srv;
    // srv.request.a = atoll(argv[1]);
    // srv.request.b = atoll(argv[2]);
    srv.request.a = 1;
    srv.request.b = 3;

    ROS_INFO("向服务端发送请求: a = %ld, b = %ld", (long)srv.request.a, (long)srv.request.b);

    if (client.call(srv))
    {
        ROS_INFO("收到响应: sum = %ld", (long)srv.response.sum);
    }
    else
    {
        ROS_ERROR("调用服务失败，请确认服务端是否已启动。");
        return 1;
    }

    return 0;
}
