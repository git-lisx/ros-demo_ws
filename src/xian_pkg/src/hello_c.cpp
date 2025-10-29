# include <ros/ros.h>

int main(int argc, char  *argv[])
{
    // 设置中文环境
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hello_c");
    ROS_INFO("Hello ROS from C++ , 你好，ROS！");
    ROS_INFO("Hello ROS from C++ , 你好，ROS！2");
    ROS_INFO("Hello ROS from C++ , 你好，ROS！3");
    return 0;
}
