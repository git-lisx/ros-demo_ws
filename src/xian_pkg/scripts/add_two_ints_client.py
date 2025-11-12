#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
from xian_pkg.srv import AddTwoInts

def add_two_ints_client(a, b):
    rospy.wait_for_service('/add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('/add_two_ints', AddTwoInts)
        resp = add_two_ints(a, b)
        return resp.sum
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

if __name__ == "__main__":
    a, b = 1, 2
    rospy.init_node('add_two_ints_client')
    rospy.loginfo("请求 %d + %d", a, b)
    print("结果:", add_two_ints_client(a, b))
