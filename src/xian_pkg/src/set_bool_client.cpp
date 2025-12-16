#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  // 设置中文环境
  // setlocale(LC_ALL, "");
  setlocale(LC_ALL, "zh_CN.UTF-8");

  ros::init(argc, argv, "set_bool_client");

  ros::service::waitForService("/set_bool");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/set_bool");
  std_srvs::SetBool srv;

  srv.request.data = true;

  if (client.call(srv))
  {
    if (srv.response.success)
    {
      ROS_INFO("服务调用成功: %s", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("服务调用失败");
    }
  }
  else
  {
    ROS_ERROR("无法调用服务 set_bool");
    return 1;
  }

  return 0;
}