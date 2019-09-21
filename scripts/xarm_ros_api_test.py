#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from xarm_ros_api.xarm_ros_api import XArmRosApi

if __name__ == '__main__':
    rospy.init_node("xarm_ros_api_test_node")
    api = XArmRosApi(move_group="xarm5", sim=False)

    api.movej([0, -0.5, 0, 0, 0], wait=True)
    api.movel(0, 0, 0.2, 0, 0, 0, wait=True)
    api.movel(0, 0, -0.2, 0, 0, 0, wait=True)
    api.movej([0, 0, 0, 0, 0], wait=True)

    # api.movej([2.888, 0.132, -1.026, 0.897, -0.00], wait=True)
    # api.movel(0, 0, 0.3, 0, 0, 0, wait=True)
    # api.movej([0.491, 0.708, -1.878, 1.158, -0.067], wait=True)
