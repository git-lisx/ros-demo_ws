#include "xian_pkg/uav_manager.h"
#include <fstream>
#include <limits>
#include <utility>
#include <vector>
#include <string>
#include <jsoncpp/json/json.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/CommandCode.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

// 为了兼容JSON解析，我们定义一个简单的结构体
struct WaypointData
{
    std::string name;
    float latitude;
    float longitude;
    float relativeAltitudeM;
    float speedMS;
    std::string vehicleAction; // TAKEOFF, LAND, etc.
    std::string cameraAction; // START_PHOTO_DISTANCE, STOP_PHOTO_DISTANCE, etc.
    double gimbalPitchDeg;
    double gimbalYawDeg;
    double cameraPhotoDistanceM;
    std::string flightPathStatus; // TRANSIT or other
};


UavManager::UavManager(std::string uav_prefix) : uav_prefix_(std::move(uav_prefix))
{
    // 初始化节点
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "mission_class_manager", ros::init_options::AnonymousName);
    }

    // 构建话题和服务名称
    const std::string state_topic = uav_prefix_ + "/mavros/state";
    const std::string waypoint_list_topic = uav_prefix_ + "/mavros/mission/waypoints";

    // 订阅状态话题
    state_sub_ = nh_.subscribe(state_topic, 10, &UavManager::stateCallback, this);

    // 初始化服务客户端
    const std::string waypoint_clear_service = uav_prefix_ + "/mavros/mission/clear";
    const std::string waypoint_push_service = uav_prefix_ + "/mavros/mission/push";
    const std::string set_mode_service = uav_prefix_ + "/mavros/set_mode";
    const std::string arming_service = uav_prefix_ + "/mavros/cmd/arming";

    waypoint_clear_client_ = nh_.serviceClient<mavros_msgs::WaypointClear>(waypoint_clear_service);
    waypoint_push_client_ = nh_.serviceClient<mavros_msgs::WaypointPush>(waypoint_push_service);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service);
}

UavManager::~UavManager() = default;

void UavManager::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void UavManager::wait_for_fcu() const
{
    ROS_INFO("等待 FCU 连接...");
    ros::Rate rate(1);
    while (ros::ok() && !current_state_.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU 已连接");
}

void UavManager::clear_mission()
{
    waypoints_.clear();

    mavros_msgs::WaypointClear srv;
    if (waypoint_clear_client_.call(srv))
    {
        if (!srv.response.success)
        {
            ROS_ERROR("清空 Mission 失败");
            return;
        }
    }
    else
    {
        ROS_ERROR("服务调用失败");
        return;
    }
    ROS_INFO("Mission 已清空");
}

std::vector<mavros_msgs::Waypoint> UavManager::download_waypoints()
{
    try
    {
        std::string waypointsTopic = uav_prefix_ + "/mavros/mission/waypoints";
        // 等待waypoint list话题可用
        ros::topic::waitForMessage<mavros_msgs::WaypointList>(waypointsTopic, ros::Duration(5));

        // 获取当前waypoint list
        mavros_msgs::WaypointListConstPtr waypoints_msg = ros::topic::waitForMessage<mavros_msgs::WaypointList>(
            waypointsTopic, ros::Duration(5));

        if (waypoints_msg != nullptr)
        {
            waypoints_ = waypoints_msg->waypoints;

            // 打印每个航点的详细信息
            for (size_t i = 0; i < waypoints_.size(); ++i)
            {
                const auto& waypoint = waypoints_[i];
                ROS_INFO("--- 航点 %lu ---", i + 1);
                ROS_INFO("  坐标系: %d", waypoint.frame);
                ROS_INFO("  命令: %d", waypoint.command);
                ROS_INFO("  是否当前航点: %s", waypoint.is_current ? "true" : "false");
                ROS_INFO("  自动继续: %s", waypoint.autocontinue ? "true" : "false");
                ROS_INFO("  参数1: %.2f", waypoint.param1);
                ROS_INFO("  参数2: %.2f", waypoint.param2);
                ROS_INFO("  参数3: %.2f", waypoint.param3);
                ROS_INFO("  参数4: %.2f", waypoint.param4);
                ROS_INFO("  纬度: %.8f", waypoint.x_lat);
                ROS_INFO("  经度: %.8f", waypoint.y_long);
                ROS_INFO("  高度: %.2f", waypoint.z_alt);
            }

            ROS_INFO("从话题读取到 %lu 个航点", waypoints_.size());
            return waypoints_;
        }
        else
        {
            ROS_ERROR("获取航点列表失败");
            return {};
        }
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR("等待航点列表话题超时: %s", e.what());
        return {};
    }
    catch (...)
    {
        ROS_ERROR("读取航点列表时发生未知错误");
        return {};
    }
}

bool UavManager::upload_waypoints()
{
    try
    {
        mavros_msgs::WaypointPush push_srv;
        push_srv.request.start_index = 0;
        push_srv.request.waypoints = waypoints_;

        if (waypoint_push_client_.call(push_srv))
        {
            ROS_INFO("上传响应 - 成功: %s, 传输数量: %d",
                     push_srv.response.success ? "true" : "false",
                     push_srv.response.wp_transfered);

            if (!push_srv.response.success)
            {
                ROS_ERROR("Mission 上传失败，传输航点数: %d", push_srv.response.wp_transfered);
                return false;
            }

            ROS_INFO("Mission 上传成功，航点数: %d", push_srv.response.wp_transfered);
            return true;
        }
        else
        {
            ROS_ERROR("Mission 上传服务调用失败");
            return false;
        }
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR("Mission 上传服务调用失败: %s", e.what());
        return false;
    }
}

bool UavManager::arm()
{
    // 检查无人机是否已解锁
    if (current_state_.armed)
    {
        ROS_INFO("无人机已经解锁");
        return true;
    }

    // 检查无人机是否准备好解锁
    if (!current_state_.connected)
    {
        ROS_ERROR("无人机未连接");
        return false;
    }

    ROS_INFO("当前模式: %s", current_state_.mode.c_str());

    // 如果当前模式不适合解锁，则切换到AUTO.LOITER模式
    if (current_state_.mode == "AUTO.RTL")
    {
        ROS_INFO("当前模式: %s，切换到AUTO.LOITER模式以便解锁", current_state_.mode.c_str());
        if (!set_mode("AUTO.LOITER"))
        {
            ROS_ERROR("无法切换到AUTO.LOITER模式，解锁失败");
            return false;
        }

        (void)ros::Duration(2).sleep(); // 等待模式切换完成
    }

    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;

    // 重试3次
    int max_retries = 3;
    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
        if (arming_client_.call(arm_srv))
        {
            if (arm_srv.response.success)
            {
                ROS_INFO("解锁成功");
                return true;
            }
            else
            {
                ROS_ERROR("第 %d 次解锁失败，错误码: %d", attempt + 1, arm_srv.response.result);
                if (attempt < max_retries - 1)
                {
                    ROS_INFO("等待2秒后重试...");
                    (void)ros::Duration(2).sleep();
                }
            }
        }
        else
        {
            ROS_ERROR("解锁服务调用失败 (尝试 %d/%d)", attempt + 1, max_retries);
            if (attempt < max_retries - 1)
            {
                ROS_INFO("等待2秒后重试...");
                (void)ros::Duration(2).sleep();
            }
        }
    }

    ROS_ERROR("多次尝试解锁失败，建议检查以下几点：");
    ROS_ERROR("1. 无人机是否已连接并就绪");
    ROS_ERROR("2. 电池电量是否充足");
    ROS_ERROR("3. 是否存在安全问题（如电机未锁定）");
    ROS_ERROR("4. 飞控是否处于合适的模式（如等待AUTO.LOITER）");
    ROS_ERROR("5. GPS是否已定位（某些配置要求GPS定位）");
    return false;
}

bool UavManager::set_mode(const std::string& mode)
{
    mavros_msgs::SetMode mode_srv;
    mode_srv.request.custom_mode = mode;

    if (set_mode_client_.call(mode_srv))
    {
        if (!mode_srv.response.mode_sent)
        {
            ROS_ERROR("模式切换失败: %s", mode.c_str());
            return false;
        }
        ROS_INFO("模式切换成功: %s", mode.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("服务调用失败: %s", mode.c_str());
        return false;
    }
}

std::vector<mavros_msgs::Waypoint> UavManager::load_ucc_air_line_waypoints(const std::string& flight_path_file)
{
    // 清空之前的航点
    waypoints_.clear();

    // 读取JSON文件
    std::ifstream file(flight_path_file);
    if (!file.is_open())
    {
        ROS_ERROR("无法打开文件: %s", flight_path_file.c_str());
        return waypoints_;
    }

    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(file, root);
    file.close();

    if (!parsingSuccessful)
    {
        ROS_ERROR("解析JSON文件失败: %s", reader.getFormattedErrorMessages().c_str());
        return waypoints_;
    }

    // 确保root是数组
    if (!root.isArray())
    {
        ROS_ERROR("JSON文件根元素不是数组");
        return waypoints_;
    }

    // 过滤掉状态为TRANSIT的临时航点
    std::vector<Json::Value> filtered_data;
    for (const auto& wp : root)
    {
        if (wp.isMember("flightPathStatus") && wp["flightPathStatus"].asString() != "TRANSIT")
        {
            filtered_data.push_back(wp);
        }
    }

    // 打印整个列表
    ROS_INFO("飞行路径点总数: %lu", filtered_data.size());

    // 获取当前GPS位置
    sensor_msgs::NavSatFixConstPtr init_gps =
        ros::topic::waitForMessage<sensor_msgs::NavSatFix>(
            uav_prefix_ + "/mavros/global_position/global", ros::Duration(10));

    if (!init_gps)
    {
        ROS_ERROR("无法获取初始GPS位置");
        return waypoints_;
    }

    bool has_takeoff = false;
    for (const auto& waypoint : filtered_data)
    {
        if (waypoint.isMember("vehicleAction") && waypoint["vehicleAction"].asString() == "TAKEOFF")
        {
            has_takeoff = true;
            break;
        }
    }

    if (!has_takeoff)
    {
        // 起飞点
        add_takeoff_waypoint(
            init_gps->latitude,
            init_gps->longitude,
            30.0); // 假设起飞高度为30米

        // 停止图像捕获命令
        add_stop_capture_waypoint();
    }

    // 打印每个航点的信息并添加到waypoints
    float distance_m = 0.0;
    for (size_t i = 0; i < filtered_data.size(); ++i)
    {
        const auto& waypoint = filtered_data[i];

        double latitude = waypoint["latitude"].asDouble(); // 纬度
        double longitude = waypoint["longitude"].asDouble(); // 经度
        double altitude = waypoint["relativeAltitudeM"].asDouble(); // 高度

        std::string name = waypoint.isMember("name")
                               ? (waypoint["name"].isNull()
                                      ? "航点_" + std::to_string(i + 1)
                                      : waypoint["name"].asString())
                               : "航点_" + std::to_string(i + 1);
        ROS_INFO("航点 %lu: %s", i + 1, name.c_str());
        ROS_INFO("  经纬度: (%.8f, %.8f) 相对高度: %.1fm", latitude, longitude, altitude);
        ROS_INFO("  速度: %.1fm/s", waypoint["speedMS"].asDouble());

        std::string vehicleAction = waypoint.isMember("vehicleAction") ? waypoint["vehicleAction"].asString() : "";

        if (vehicleAction == "TAKEOFF")
        {
            // 起飞点
            add_takeoff_waypoint(latitude, longitude, altitude);
            // 停止图像捕获命令
            add_stop_capture_waypoint();
        }
        else if (vehicleAction == "LAND")
        {
            add_land_waypoint_at_position(latitude, longitude, altitude);
        }
        else
        {
            add_nav_waypoint(latitude, longitude, altitude);
        }

        // 设置速度
        add_change_speed_waypoint(1, waypoint["speedMS"].asFloat());

        // 云台控制命令
        if (waypoint.isMember("gimbalPitchDeg") && !waypoint["gimbalPitchDeg"].isNull())
        {
            float pitch_deg = waypoint["gimbalPitchDeg"].asFloat();
            float yaw_deg = waypoint.isMember("gimbalYawDeg") && !waypoint["gimbalYawDeg"].isNull()
                                ? waypoint["gimbalYawDeg"].asFloat()
                                : 0.0;
            add_set_gimbal_waypoint(pitch_deg, yaw_deg);
        }

        std::string cameraAction = waypoint.isMember("cameraAction")
                                       ? (waypoint["cameraAction"].isNull() ? "" : waypoint["cameraAction"].asString())
                                       : "";
        ROS_INFO("  相机动作: %s", cameraAction.c_str());

        if (waypoint.isMember("cameraPhotoDistanceM") && !waypoint["cameraPhotoDistanceM"].isNull())
        {
            distance_m = waypoint["cameraPhotoDistanceM"].asFloat();
        }

        if (cameraAction == "STOP_PHOTO_DISTANCE")
        {
            // 停止图像捕获
            add_stop_capture_waypoint();
            // 重置云台
            add_set_gimbal_waypoint(0, 0);
        }
        else if (cameraAction == "START_PHOTO_DISTANCE" || cameraAction.empty())
        {
            //  相机触发距离设置
            add_set_cam_trigger_dist_waypoint(distance_m);
        }

        ROS_INFO("  %s", std::string(40, '-').c_str());
    }

    bool has_land = false;
    for (const auto& waypoint : filtered_data)
    {
        if (waypoint.isMember("vehicleAction") && waypoint["vehicleAction"].asString() == "LAND")
        {
            has_land = true;
            break;
        }
    }

    if (!has_land)
    {
        // 可以使用起飞点位置降落
        add_land_waypoint_at_position(
            init_gps->latitude,
            init_gps->longitude,
            0.0);
    }

    return waypoints_;
}

void UavManager::add_takeoff_waypoint(double latitude, double longitude, double altitude)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current = true;
    wp.autocontinue = true;
    wp.param1 = 0.0;
    wp.param2 = 0.0;
    wp.param3 = 0.0;
    wp.param4 = std::numeric_limits<double>::quiet_NaN();
    wp.x_lat = latitude;
    wp.y_long = longitude;
    wp.z_alt = altitude;
    waypoints_.push_back(wp);
}

void UavManager::add_land_waypoint_at_position(double latitude, double longitude, double altitude)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp.command = mavros_msgs::CommandCode::NAV_LAND;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = 0.0;
    wp.param2 = 0.0;
    wp.param3 = 0.0;
    wp.param4 = std::numeric_limits<double>::quiet_NaN();
    wp.x_lat = latitude;
    wp.y_long = longitude;
    wp.z_alt = altitude;
    waypoints_.push_back(wp);
}

void UavManager::add_set_cam_trigger_dist_waypoint(float distance_m)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command = mavros_msgs::CommandCode::DO_SET_CAM_TRIGG_DIST;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = distance_m;
    wp.param2 = 0.0;
    wp.param3 = 0.0;
    wp.param4 = 0.0;
    wp.x_lat = 0.0;
    wp.y_long = 0.0;
    wp.z_alt = 0.0;
    waypoints_.push_back(wp);
}

void UavManager::add_change_speed_waypoint(float speed_type, float speed_m_s, float acceleration)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command = mavros_msgs::CommandCode::DO_CHANGE_SPEED;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = speed_type;
    wp.param2 = speed_m_s;
    wp.param3 = acceleration;
    wp.param4 = 0.0;
    wp.x_lat = 0.0;
    wp.y_long = 0.0;
    wp.z_alt = 0.0;
    waypoints_.push_back(wp);
}

void UavManager::add_nav_waypoint(double latitude, double longitude, double altitude)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = 0.0;
    wp.param2 = 0.0;
    wp.param3 = 0.0;
    wp.param4 = std::numeric_limits<double>::quiet_NaN();
    wp.x_lat = latitude;
    wp.y_long = longitude;
    wp.z_alt = altitude;
    waypoints_.push_back(wp);
}

void UavManager::add_set_gimbal_waypoint(float pitch_deg, float yaw_deg)
{
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = pitch_deg;
    wp.param2 = 0.0;
    wp.param3 = yaw_deg;
    wp.param4 = 0.0;
    wp.x_lat = 0.0;
    wp.y_long = 0.0;
    wp.z_alt = 2.00; // 根据项目规范，此参数通常被忽略
    waypoints_.push_back(wp);
}

void UavManager::add_stop_capture_waypoint()
{
    // 航点 - 相机触发距离设置
    mavros_msgs::Waypoint wp1;
    wp1.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp1.command = mavros_msgs::CommandCode::DO_SET_CAM_TRIGG_DIST;
    wp1.is_current = false;
    wp1.autocontinue = true;
    wp1.param1 = 0.0;
    wp1.param2 = 0.0;
    wp1.param3 = 0.0;
    wp1.param4 = 0.0;
    wp1.x_lat = 0.0;
    wp1.y_long = 0.0;
    wp1.z_alt = 0.0;
    waypoints_.push_back(wp1);

    // 航点 - 停止图像捕获命令
    mavros_msgs::Waypoint wp2;
    wp2.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp2.command = mavros_msgs::CommandCode::IMAGE_STOP_CAPTURE;
    wp2.is_current = false;
    wp2.autocontinue = true;
    wp2.param1 = 0.0;
    wp2.param2 = std::numeric_limits<double>::quiet_NaN();
    wp2.param3 = std::numeric_limits<double>::quiet_NaN();
    wp2.param4 = std::numeric_limits<double>::quiet_NaN();
    wp2.x_lat = -2147483648.0;
    wp2.y_long = -2147483648.0;
    wp2.z_alt = std::numeric_limits<double>::quiet_NaN();
    waypoints_.push_back(wp2);
}

void UavManager::execute_mission(const std::string& flight_path_file)
{
    // 等待飞控单元(FCU)连接
    wait_for_fcu();

    clear_mission();

    // 加载航点
    load_ucc_air_line_waypoints(flight_path_file);

    // 上传航点
    bool success = upload_waypoints();
    if (!success)
    {
        ROS_ERROR("任务上传失败，退出程序");
        return;
    }
    (void)ros::Duration(2).sleep();

    // 解锁无人机，如果失败则退出程序
    if (!arm())
    {
        ROS_ERROR("解锁失败，程序退出");
        return;
    }
    (void)ros::Duration(1).sleep();

    // 切换到AUTO.MISSION模式，启动任务
    set_mode("AUTO.MISSION");
    (void)ros::Duration(1).sleep();

    ROS_INFO("任务执行中");
}
