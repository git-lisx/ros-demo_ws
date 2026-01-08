#ifndef UAV_MANAGER_H
#define UAV_MANAGER_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <vector>
#include <string>

/**
 * @brief UAV管理器类，用于控制无人机执行任务
 * 
 * 该类提供了与MAVROS交互的接口，包括无人机状态监控、任务上传下载、
 * 航点管理、模式设置和无人机控制等功能
 */
class UavManager
{
public:
    /**
     * @brief 构造函数
     * @param uav_prefix 无人机标识前缀，用于多无人机系统
     */
    explicit UavManager(std::string uav_prefix = "");

    /**
     * @brief 析构函数
     */
    ~UavManager();

    /**
     * @brief 无人机状态回调函数
     * @param msg 无人机当前状态消息
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    /**
     * @brief 等待飞控连接
     * 等待与飞控建立连接，直到接收到飞控状态消息
     */
    void wait_for_fcu() const;

    /**
     * @brief 清空当前任务
     * 清除无人机上的所有航点任务
     */
    void clear_mission();

    /**
     * @brief 下载航点列表
     * 从无人机获取当前的航点列表
     * @return 航点列表
     */
    std::vector<mavros_msgs::Waypoint> download_waypoints();

    /**
     * @brief 上传航点列表到无人机
     * 将当前航点列表上传到无人机
     * @return 上传是否成功
     */
    bool upload_waypoints();

    /**
     * @brief 无人机解锁
     * 执行无人机解锁操作
     * @return 解锁是否成功
     */
    bool arm();

    /**
     * @brief 设置无人机飞行模式
     * @param mode 飞行模式名称（如"GUIDED", "AUTO", "LOITER"等）
     * @return 设置是否成功
     */
    bool set_mode(const std::string& mode);

    /**
     * @brief 从文件加载航点任务
     * 从JSON文件中加载UCC航点航线任务
     * @param flight_path_file 航线文件路径
     * @return 载入的航点列表
     */
    std::vector<mavros_msgs::Waypoint> load_ucc_air_line_waypoints(const std::string& flight_path_file);

    /**
     * @brief 执行任务
     * 加载并执行指定的飞行任务
     * @param flight_path_file 任务文件路径，默认为"ucc-air-line2.json"
     */
    void execute_mission(const std::string& flight_path_file = "ucc-air-line2.json");

private:
    ros::NodeHandle nh_; ///< ROS节点句柄
    std::string uav_prefix_; ///< 无人机标识前缀，用于多无人机系统
    mavros_msgs::State current_state_; ///< 无人机当前状态
    std::vector<mavros_msgs::Waypoint> waypoints_; ///< 存储航点列表

    // 订阅者
    ros::Subscriber state_sub_; ///< 无人机状态订阅者

    // 服务客户端
    ros::ServiceClient waypoint_clear_client_; ///< 航点清除服务客户端
    ros::ServiceClient waypoint_push_client_; ///< 航点推送服务客户端
    ros::ServiceClient set_mode_client_; ///< 模式设置服务客户端
    ros::ServiceClient arming_client_; ///< 解锁服务客户端

    // 话题订阅
    ros::Subscriber waypoint_list_sub_; ///< 航点列表订阅者

    // 私有辅助函数

    /**
     * @brief 添加起飞航点
     * @param latitude 起飞点纬度
     * @param longitude 起飞点经度
     * @param altitude 起飞点高度，默认为0.0
     */
    void add_takeoff_waypoint(double latitude, double longitude, double altitude = 0.0);

    /**
     * @brief 添加降落航点
     * @param latitude 降落点纬度
     * @param longitude 降落点经度
     * @param altitude 降落点高度，默认为0.0
     */
    void add_land_waypoint_at_position(double latitude, double longitude, double altitude = 0.0);

    /**
     * @brief 添加相机触发距离设置航点
     * @param distance_m 相机触发距离（米）
     */
    void add_set_cam_trigger_dist_waypoint(float distance_m);

    /**
     * @brief 添加速度变更航点
     * @param speed_type 速度类型，默认为1
     * @param speed_m_s 速度值（米/秒），默认为4.0
     * @param acceleration 加速度，默认为-1.0（使用默认值）
     */
    void add_change_speed_waypoint(float speed_type = 1, float speed_m_s = 4.0, float acceleration = -1.0);

    /**
     * @brief 添加导航航点
     * @param latitude 纬度
     * @param longitude 经度
     * @param altitude 高度
     */
    void add_nav_waypoint(double latitude, double longitude, double altitude);

    /**
     * @brief 添加云台控制航点
     * @param pitch_deg 俯仰角度，默认-90度（向下看）
     * @param yaw_deg 偏航角度，默认0度
     */
    void add_set_gimbal_waypoint(float pitch_deg = -90.00, float yaw_deg = 0.00);

    /**
     * @brief 添加停止拍摄航点
     * 停止相机拍摄任务
     */
    void add_stop_capture_waypoint();
};

#endif // UAV_MANAGER_H
