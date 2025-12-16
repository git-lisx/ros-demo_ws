#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

def callback(data):
    # 当收到消息时，这个函数会被调用
    rospy.loginfo("收到: %s", data.data)

def listener():
    # 初始化节点
    rospy.init_node('listener', anonymous=False)
    
    # 创建订阅者，订阅 'chatter' 话题
    # 当有新消息时，调用 callback 函数
    rospy.Subscriber('chatter', String, callback)
    
    rospy.loginfo("开始监听消息...")
    
    # 进入循环，等待回调函数被触发
    # spin() 保持节点运行，直到被关闭
    rospy.spin()

if __name__ == '__main__':
    listener()