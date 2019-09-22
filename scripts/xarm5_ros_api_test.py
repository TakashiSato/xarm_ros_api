#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import argparse
from geometry_msgs.msg import Pose
from xarm_ros_api.xarm_ros_api import XArmRosApi

def parse_arguments():
    parser = argparse.ArgumentParser(
        description='XArm5 pick and place demo.')
    parser.add_argument('--sim', action='store_true',
                        help='Simulation mode')
    args, unknown = parser.parse_known_args()
    return args

def main(args):
    rospy.init_node("xarm_ros_api_test_node")
    ee_link = 'link5'
    api = XArmRosApi(move_group="xarm5", ee_link=ee_link, sim=args.sim)

    try:
        # setup
        api.clear_error()
        api.enable(8)
        api.set_mode(1)
        api.set_state(0)

        # home
        home_pose = api.get_fk(ee_link, [0, 0, 0, 0, 0])
        api.movej([0, 0, 0, 0, 0], wait=True)
        api.movel_rel(home_pose, 0, 0, 0.1, 0, 0, 0, wait=True)

        # pick
        pick_pose = api.get_fk(ee_link, [2.888, 0.140, -1.026, 0.896, -0.005])
        api.movep_rel(pick_pose, 0, 0, 0.1, 0, 0, 0, wait=True)
        api.movel(pick_pose, wait=True)
        api.movel_rel(pick_pose, 0, 0, 0.1, 0, 0, 0, wait=True)

        # back home
        api.movep_rel(home_pose, 0, 0, 0.1, 0, 0, 0, wait=True)
        api.movej([0, 0, 0, 0, 0], wait=True)

    except Exception as e:
        rospy.logerr(e)
    finally:
        pass
        # api.disable(8)
    rospy.loginfo('teardown')

if __name__ == '__main__':
    args = parse_arguments()
    main(args)
