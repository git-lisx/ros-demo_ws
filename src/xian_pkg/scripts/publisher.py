#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

def talker():
    # 初始化节点，名称必须唯一
    rospy.init_node('publisher', anonymous=False)
    
    # 创建发布者，发布到 'chatter' 话题，消息类型为 String
    # queue_size 是缓冲区大小
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # 设置发布频率（Hz）
    rate = rospy.Rate(10)  # 1 Hz = 每秒1次
    
    count = 1
    rospy.loginfo("开始发布消息...")
    
    # 循环发布，直到节点被关闭
    while not rospy.is_shutdown():
        # 创建要发布的消息
        hello_str = "Hello ROS! 这是第 {} 条消息".format(count)
        
        # 发布消息
        pub.publish(hello_str)
        
        # 打印日志
        rospy.loginfo("发布: %s", hello_str)
        
        count += 1
        rate.sleep()  # 等待，保持发布频率

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass