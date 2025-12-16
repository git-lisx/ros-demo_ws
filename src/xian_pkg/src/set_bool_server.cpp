#include "ros/ros.h"
#include "std_srvs/SetBool.h"

bool setBoolCallback(std_srvs::SetBool::Request &req,
                     std_srvs::SetBool::Response &res)
{
  if (req.data)
  {
    res.success = true;
    res.message = "成功设置为 true";
    ROS_INFO("服务被调用，参数为 true");
  }
  else
  {
    res.success = true;
    res.message = "成功设置为 false";
    ROS_INFO("服务被调用，参数为 false");
  }

  return true;
}

int main(int argc, char **argv)
{
  // 设置中文环境
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "set_bool_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/set_bool", setBoolCallback);
  ROS_INFO("准备接收SetBool服务调用。");
  ros::spin();

  return 0;
}