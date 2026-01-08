#include "xian_pkg/uav_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    // 设置中文环境
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "uav_manager_node", ros::init_options::AnonymousName);
    
    // 创建UAV管理器实例
    UavManager uav_manager("/typhoon_h480_0");

    // 执行任务
    uav_manager.execute_mission("/home/dataexa/lisx/ros-demo_ws/src/xian_pkg/scripts/ucc-air-line2.json");
    // uav_manager.download_waypoints();
    // ros::spin();

    return 0;
}