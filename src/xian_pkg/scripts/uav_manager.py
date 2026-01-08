#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
使用类封装的无人机任务管理模块
"""
import json
import time
from typing import List

import rospy
from mavros_msgs.msg import State, Waypoint, CommandCode, WaypointList
from mavros_msgs.srv import (
    WaypointClear,
    WaypointPush,
    SetMode,
    CommandBool
)
from sensor_msgs.msg import NavSatFix


class UavManager:
    """
    无人机管理类，封装了航点上传、任务执行等核心功能
    """

    def __init__(self, uav_prefix=""):
        """
        初始化任务管理器
        
        Args:
            uav_prefix (str): 无人机前缀，用于构建服务和话题名称
        """
        rospy.init_node("mission_class_manager", anonymous=True)
        self.uav_prefix = uav_prefix
        self.current_state = State()
        self.waypoints: List[Waypoint] = []
        rospy.Subscriber(f"{self.uav_prefix}/mavros/state", State, self.state_callback)

    def state_callback(self, msg):
        """
        状态回调函数，更新无人机当前状态
        
        Args:
            msg: 来自mavros的状态消息
        """
        self.current_state = msg

    def wait_for_fcu(self):
        """
        等待飞控单元(FCU)连接
        """
        rospy.loginfo("等待 FCU 连接...")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("FCU 已连接")

    def clear_mission(self):
        """
        清空当前任务
        """
        self.waypoints = []
        rospy.wait_for_service(f"{self.uav_prefix}/mavros/mission/clear")
        srv = rospy.ServiceProxy(f"{self.uav_prefix}/mavros/mission/clear", WaypointClear)
        res = srv()
        if not res.success:
            rospy.logerr("清空 Mission 失败")
            return
        rospy.loginfo("Mission 已清空")

    def download_waypoints(self):
        """
        从航点列表话题读取已下载的航点
        """
        try:
            # 等待航点列表话题可用
            rospy.wait_for_message(f"{self.uav_prefix}/mavros/mission/waypoints", WaypointList, timeout=5)

            # 订阅航点列表话题
            waypoints_msg = rospy.wait_for_message(f"{self.uav_prefix}/mavros/mission/waypoints", WaypointList)

            self.waypoints = waypoints_msg.waypoints

            # 打印每个航点的详细信息
            for i, waypoint in enumerate(self.waypoints):
                rospy.loginfo("--- 航点 %d ---", i + 1)
                rospy.loginfo("  坐标系: %d", waypoint.frame)
                rospy.loginfo("  命令: %d", waypoint.command)
                rospy.loginfo("  是否当前航点: %s", waypoint.is_current)
                rospy.loginfo("  自动继续: %s", waypoint.autocontinue)
                rospy.loginfo("  参数1: %.2f", waypoint.param1)
                rospy.loginfo("  参数2: %.2f", waypoint.param2)
                rospy.loginfo("  参数3: %.2f", waypoint.param3)
                rospy.loginfo("  参数4: %.2f", waypoint.param4)
                rospy.loginfo("  纬度: %.8f", waypoint.x_lat)
                rospy.loginfo("  经度: %.8f", waypoint.y_long)
                rospy.loginfo("  高度: %.2f", waypoint.z_alt)

            rospy.loginfo("从话题读取到 %d 个航点", len(self.waypoints))
            return self.waypoints
        except rospy.ROSException as e:
            rospy.logerr("等待航点列表话题超时: %s", e)
            return []
        except Exception as e:
            rospy.logerr("读取航点列表时发生错误: %s", e)
            return []

    def upload_waypoints(self):
        """
        上传waypoints


        Returns:
            bool: 上传是否成功
        """
        try:
            rospy.wait_for_service(f"{self.uav_prefix}/mavros/mission/push")
            rospy.loginfo("服务可用，开始上传 %d 个航点", len(self.waypoints))
            push_srv = rospy.ServiceProxy(f"{self.uav_prefix}/mavros/mission/push", WaypointPush)
            res = push_srv(start_index=0, waypoints=self.waypoints)

            rospy.loginfo("上传响应 - 成功: %s, 传输数量: %d", res.success, res.wp_transfered)

            if not res.success:
                rospy.logerr("Mission 上传失败，传输航点数: %d", res.wp_transfered)
                return False

            rospy.loginfo("Mission 上传成功，航点数: %d", res.wp_transfered)
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Mission 上传服务调用失败: %s", e)
            return False

    def arm(self):
        """
        解锁无人机，包含状态检查、模式切换和重试机制

        Returns:
            bool: 解锁是否成功
        """
        # 等待服务可用
        rospy.wait_for_service(f"{self.uav_prefix}/mavros/cmd/arming")

        # 检查无人机是否已解锁
        if self.current_state.armed:
            rospy.loginfo("无人机已经解锁")
            return True

        # 检查无人机是否准备好解锁
        if not self.current_state.connected:
            rospy.logerr("无人机未连接")
            return False

        rospy.loginfo("当前模式: %s", self.current_state.mode)
        # # 如果当前模式不适合解锁，则切换到AUTO.LOITER模式
        if self.current_state.mode == "AUTO.RTL":
            rospy.loginfo("当前模式: %s，切换到AUTO.LOITER模式以便解锁", self.current_state.mode)
            if not self.set_mode("AUTO.LOITER"):
                rospy.logerr("无法切换到AUTO.LOITER模式，解锁失败")
                return False

            rospy.sleep(2)  # 等待模式切换完成

        arm_srv = rospy.ServiceProxy(f"{self.uav_prefix}/mavros/cmd/arming", CommandBool)

        # 重试3次
        max_retries = 3
        for attempt in range(max_retries):
            try:
                res = arm_srv(True)
                if res.success:
                    rospy.loginfo(f"解锁成功")
                    return True
                else:
                    rospy.logerr(f"第 {attempt + 1} 次解锁失败，错误码: {res.result}")
                    if attempt < max_retries - 1:
                        rospy.loginfo(f"等待2秒后重试...")
                        time.sleep(2)
            except rospy.ServiceException as e:
                rospy.logerr(f"解锁服务调用失败 (尝试 {attempt + 1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    rospy.loginfo(f"等待2秒后重试...")
                    time.sleep(2)

        rospy.logerr("多次尝试解锁失败，建议检查以下几点：")
        rospy.logerr("1. 无人机是否已连接并就绪")
        rospy.logerr("2. 电池电量是否充足")
        rospy.logerr("3. 是否存在安全问题（如电机未锁定）")
        rospy.logerr("4. 飞控是否处于合适的模式（如等待AUTO.LOITER）")
        rospy.logerr("5. GPS是否已定位（某些配置要求GPS定位）")
        return False

    def set_mode(self, mode):
        """
        设置无人机飞行模式

        Args:
            mode: 模式名称

        Returns:
            bool: 设置是否成功
        """
        rospy.wait_for_service(f"{self.uav_prefix}/mavros/set_mode")
        mode_srv = rospy.ServiceProxy(f"{self.uav_prefix}/mavros/set_mode", SetMode)
        res = mode_srv(custom_mode=mode)
        if not res.mode_sent:
            rospy.logerr("模式切换失败: %s", mode)
            return False
        rospy.loginfo("模式切换成功: %s", mode)
        return True

    def load_ucc_air_line_waypoints(self, flight_path_file):
        """
        从JSON文件读取航点数据并转换为MAVROS航点格式
        
        Args:
            flight_path_file (str): 包含飞行路径的JSON文件路径
            
        Returns:
            List[Waypoint]: 转换后的MAVROS航点列表
        """
        # 清空之前的航点
        self.waypoints = []

        # 读取JSON文件
        with open(flight_path_file, 'r', encoding='utf-8') as file:
            flight_path_data = json.load(file)

        # 过滤掉状态为TRANSIT的临时航点
        flight_path_data = [wp for wp in flight_path_data if wp['flightPathStatus'] != 'TRANSIT']

        # 打印整个列表
        print("飞行路径点总数:", len(flight_path_data))

        # 获取当前GPS位置
        init_gps = rospy.wait_for_message(f"{self.uav_prefix}/mavros/global_position/global", NavSatFix)
        has_takeoff = any(waypoint.get('vehicleAction') == 'TAKEOFF' for waypoint in flight_path_data)
        if not has_takeoff:
            # 起飞点
            self.add_takeoff_waypoint(
                latitude=init_gps.latitude,
                longitude=init_gps.longitude,
                altitude=30.0)

            # 停止图像捕获命令
            self.add_stop_capture_waypoint()

        # 打印每个航点的信息
        distance_m = 0.0
        for i, waypoint in enumerate(flight_path_data):
            print(f"航点 {i + 1}: {waypoint['name'] or f'航点_{i + 1}'}")
            print(
                f"  经纬度: ({waypoint['latitude']}, {waypoint['longitude']}) 相对高度: {waypoint['relativeAltitudeM']}m")
            print(f"  速度: {waypoint['speedMS']}m/s")
            if waypoint['vehicleAction'] == 'TAKEOFF':
                # 起飞点
                self.add_takeoff_waypoint(
                    latitude=waypoint['latitude'],
                    longitude=waypoint['longitude'],
                    altitude=waypoint['relativeAltitudeM'])
                # 停止图像捕获命令
                self.add_stop_capture_waypoint()
            elif waypoint['vehicleAction'] == 'LAND':
                self.add_land_waypoint_at_position(
                    latitude=waypoint['latitude'],
                    longitude=waypoint['longitude'],
                    altitude=waypoint['relativeAltitudeM'])
            else:
                self.add_nav_waypoint(latitude=waypoint['latitude'], longitude=waypoint['longitude'],
                                      altitude=waypoint['relativeAltitudeM'])

            # 设置速度
            self.add_change_speed_waypoint(speed_type=1, speed_m_s=waypoint['speedMS'])

            # 云台控制命令
            if waypoint['gimbalPitchDeg'] is not None or waypoint['gimbalYawDeg'] is not None:
                self.add_set_gimbal_waypoint(pitch_deg=waypoint['gimbalPitchDeg'], yaw_deg=waypoint['gimbalYawDeg'])

            print(f"  相机动作: {waypoint['cameraAction']}")
            if waypoint['cameraPhotoDistanceM']:
                distance_m = waypoint['cameraPhotoDistanceM']
            if waypoint['cameraAction'] == 'START_PHOTO_DISTANCE':
                # 相机触发距离设置
                self.add_set_cam_trigger_dist_waypoint(distance_m=distance_m)
            elif waypoint['cameraAction'] == 'STOP_PHOTO_DISTANCE':
                # 停止图像捕获
                self.add_stop_capture_waypoint()
                self.add_set_gimbal_waypoint(pitch_deg=0)
            elif waypoint['cameraAction'] is None:
                # 根据需求，每个航点增加相机触发距离
                self.add_set_cam_trigger_dist_waypoint(distance_m=distance_m)

            print("-" * 40)

        has_land = any(waypoint.get('vehicleAction') == 'LAND' for waypoint in flight_path_data)
        if not has_land:
            # 可以使用起飞点位置降落
            self.add_land_waypoint_at_position(
                latitude=init_gps.latitude,
                longitude=init_gps.longitude,
                altitude=0.0)

        return self.waypoints

    def add_takeoff_waypoint(self, latitude: float, longitude: float, altitude: float = 0.0):
        """
        添加起飞航点
        
        Args:
            latitude: 纬度
            longitude: 经度
            altitude: 高度
        """
        wp1 = Waypoint()
        wp1.frame = Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT
        wp1.command = CommandCode.NAV_TAKEOFF
        wp1.is_current = True
        wp1.autocontinue = True
        wp1.param1 = 0.0
        wp1.param2 = 0.0
        wp1.param3 = 0.0
        wp1.param4 = float('nan')
        wp1.x_lat = latitude
        wp1.y_long = longitude
        wp1.z_alt = altitude
        self.waypoints.append(wp1)

    def add_land_waypoint_at_position(self, latitude: float, longitude: float, altitude: float = 0.0):
        """
        在指定位置添加降落航点
        
        Args:
            latitude: 纬度
            longitude: 经度
            altitude: 高度
        """
        wp_land = Waypoint()
        wp_land.frame = Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT
        wp_land.command = CommandCode.NAV_LAND
        wp_land.is_current = False
        wp_land.autocontinue = True
        wp_land.param1 = 0.0
        wp_land.param2 = 0.0
        wp_land.param3 = 0.0
        wp_land.param4 = float('nan')
        wp_land.x_lat = latitude
        wp_land.y_long = longitude
        wp_land.z_alt = altitude
        self.waypoints.append(wp_land)

    def add_set_cam_trigger_dist_waypoint(self, distance_m: float):
        """
        添加相机触发距离设置航点
        
        Args:
            distance_m: 触发距离(米)
        """
        wp4 = Waypoint()
        wp4.frame = Waypoint.FRAME_MISSION
        wp4.command = CommandCode.DO_SET_CAM_TRIGG_DIST
        wp4.is_current = False
        wp4.autocontinue = True
        wp4.param1 = distance_m
        wp4.param2 = 0.0
        wp4.param3 = 0.0
        wp4.param4 = 0.0
        wp4.x_lat = 0.0
        wp4.y_long = 0.0
        wp4.z_alt = 0.0
        self.waypoints.append(wp4)

    def add_change_speed_waypoint(self, speed_type: int = 1, speed_m_s: float = 4.0,
                                  acceleration: float = -1.0):
        """
        添加改变飞行速度的航点
        
        Args:
            speed_type: 速度类型 (0=空速, 1=地速)
            speed_m_s: 速度值 (m/s)
            acceleration: 加速度 (m/s²), -1表示使用默认值
        """
        wp_speed = Waypoint()
        wp_speed.frame = Waypoint.FRAME_MISSION
        wp_speed.command = CommandCode.DO_CHANGE_SPEED
        wp_speed.is_current = False
        wp_speed.autocontinue = True
        wp_speed.param1 = float(speed_type)
        wp_speed.param2 = speed_m_s
        wp_speed.param3 = acceleration
        wp_speed.param4 = 0.0
        wp_speed.x_lat = 0.0
        wp_speed.y_long = 0.0
        wp_speed.z_alt = 0.0
        self.waypoints.append(wp_speed)

    def add_nav_waypoint(self, latitude: float, longitude: float, altitude: float):
        """
        添加导航航点
        
        Args:
            latitude: 纬度
            longitude: 经度
            altitude: 高度
        """
        wp2 = Waypoint()
        wp2.frame = Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT
        wp2.command = CommandCode.NAV_WAYPOINT
        wp2.is_current = False
        wp2.autocontinue = True
        wp2.param1 = 0.0
        wp2.param2 = 0.0
        wp2.param3 = 0.0
        wp2.param4 = float('nan')
        wp2.x_lat = latitude
        wp2.y_long = longitude
        wp2.z_alt = altitude
        self.waypoints.append(wp2)

    def add_set_gimbal_waypoint(self, pitch_deg: float = -90.00, yaw_deg: float = 0.00):
        """
        添加云台控制航点
        
        Args:
            pitch_deg: 俯仰角（度）
            yaw_deg: 偏航角（度）
        """
        wp3 = Waypoint()
        wp3.frame = Waypoint.FRAME_MISSION
        wp3.command = CommandCode.DO_MOUNT_CONTROL
        wp3.is_current = False
        wp3.autocontinue = True
        wp3.param1 = pitch_deg if pitch_deg else 0.0
        wp3.param2 = 0.0
        wp3.param3 = yaw_deg if yaw_deg else 0.0
        wp3.param4 = 0.0
        wp3.x_lat = 0.0
        wp3.y_long = 0.0
        wp3.z_alt = 2.00  # 根据项目规范，此参数通常被忽略
        self.waypoints.append(wp3)

    def add_stop_capture_waypoint(self):
        """
        添加停止图像捕获航点
        """
        # 航点7 - 相机触发距离设置
        wp7 = Waypoint()
        wp7.frame = Waypoint.FRAME_MISSION
        wp7.command = CommandCode.DO_SET_CAM_TRIGG_DIST
        wp7.is_current = False
        wp7.autocontinue = True
        wp7.param1 = 0.0
        wp7.param2 = 0.0
        wp7.param3 = 0.0
        wp7.param4 = 0.0
        wp7.x_lat = 0.0
        wp7.y_long = 0.0
        wp7.z_alt = 0.0
        self.waypoints.append(wp7)

        # 航点8 - 停止图像捕获命令
        wp8 = Waypoint()
        wp8.frame = Waypoint.FRAME_MISSION
        wp8.command = CommandCode.IMAGE_STOP_CAPTURE
        wp8.is_current = False
        wp8.autocontinue = True
        wp8.param1 = 0.0
        wp8.param2 = float('nan')
        wp8.param3 = float('nan')
        wp8.param4 = float('nan')
        wp8.x_lat = -2147483648.0
        wp8.y_long = -2147483648.0
        wp8.z_alt = float('nan')
        self.waypoints.append(wp8)

    def execute_mission(self, flight_path_file='ucc-air-line2.json'):
        """
        执行完整的无人机任务
        
        Args:
            flight_path_file (str): 包含飞行路径的JSON文件路径
        """
        # 等待飞控单元(FCU)连接
        self.wait_for_fcu()

        self.clear_mission()

        # 加载航点
        self.load_ucc_air_line_waypoints(flight_path_file)

        # 上传航点
        success = self.upload_waypoints()
        if not success:
            rospy.logerr("任务上传失败，退出程序")
            return
        rospy.sleep(2)

        # 解锁无人机，如果失败则退出程序
        if not self.arm():
            rospy.logerr("解锁失败，程序退出")
            return
        rospy.sleep(1)

        # 切换到AUTO.MISSION模式，启动任务
        self.set_mode("AUTO.MISSION")
        rospy.sleep(1)

        rospy.loginfo("任务执行中")


if __name__ == "__main__":
    """
    主函数，演示如何使用MissionManager类
    """

    # 创建任务管理器实例
    uav_manager = UavManager(uav_prefix="/typhoon_h480_0")

    # 执行任务
    uav_manager.execute_mission()
