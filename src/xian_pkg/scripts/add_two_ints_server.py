#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from xian_pkg.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    rospy.loginfo("收到请求: a=%d, b=%d", req.a, req.b)
    result = req.a + req.b
    rospy.loginfo("响应结果: sum=%d", result)
    return AddTwoIntsResponse(result)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('/add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("已准备好相加两个整数（Python 服务）。")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
