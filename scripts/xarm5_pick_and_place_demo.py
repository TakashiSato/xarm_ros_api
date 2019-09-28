#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import argparse
import tf
import numpy as np
from xarm_ros_api.xarm_ros_api import XArmRosApi
from moveit_python import MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface
from moveit_msgs.msg import Grasp, PlaceLocation
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

def parse_arguments():
    parser = argparse.ArgumentParser(
        description='XArm5 pick and place demo.')
    parser.add_argument('--sim', action='store_true',
                        help='Simulation mode')
    args, unknown = parser.parse_known_args()
    return args

class PickPlaceDemo():

    def __init__(self):
        self._scene = PlanningSceneInterface('base_link')
        self._pickplace = PickPlaceInterface('xarm5', 'gripper', verbose=True)
        self._move_group = MoveGroupInterface('xarm5', 'base_link')

    def setup_scene(self):
        # remove previous objects
        for name in self._scene.getKnownCollisionObjects():
            self._scene.removeCollisionObject(name, False)
        for name in self._scene.getKnownAttachedObjects():
            self._scene.removeAttachedObject(name, False)
        self._scene.waitForSync()

        self.__addBoxWithOrientation('box', 0.25, 0.25, 0.25, -0.45, 0.1, 0.125, 0, 0, np.radians(45), wait=False)
        self._scene.waitForSync()

    def __addBoxWithOrientation(self, name, size_x, size_y, size_z, x, y, z, roll, pitch, yaw, wait=True):
        s = SolidPrimitive()
        s.dimensions = [size_x, size_y, size_z]
        s.type = s.BOX

        ps = PoseStamped()
        ps.header.frame_id = self._scene._fixed_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        self._scene.addSolidPrimitive(name, s, ps.pose, wait)

    def pickup(self):
        g = Grasp()
        self._pickplace.pickup("box", [g], support_name = "box")


def setup_scene():
    scene = PlanningSceneInterface('base_link')
    scene.removeCollisionObject('box')
    scene.addCube('box', 0.25, -0.45, 0.1, 0.125)

    pick_place = PickPlaceInterface('xarm5', 'gripper', verbose=True)
    grasp = Grasp()

    # fill in g
    # setup object named object_name using PlanningSceneInterface
    pick_place.pickup('box', [grasp], support_name = 'supporting_surface')

    place_loc = PlaceLocation()
    # fill in l
    pick_place.place('box' [place_loc], goal_is_eef = True, support_name = 'supporting_surface')


def main(args):
    rospy.init_node('xarm_ros_api_test_node')
    # setup_scene()
    demo = PickPlaceDemo()
    demo.setup_scene()
    demo.pickup()
    rospy.loginfo('end')
    return

    ee_link = 'link5'
    api = XArmRosApi(move_group='xarm5', ee_link=ee_link, sim=args.sim)

    try:
        # setup
        api.clear_error()
        api.set_digital_io(1, 0)
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
        api.set_digital_io(1, 1)
        rospy.sleep(0.2)
        api.movel_rel(pick_pose, 0, 0, 0.1, 0, 0, 0, wait=True)
        api.movep_rel(pick_pose, 0.1, 0, 0.1, 0, 0, 0, wait=True)

        # ready
        ready_positions = api.get_current_joint_positions()
        ready_positions[0] = 0.491
        api.movej(ready_positions, wait=True)

        # place
        place_pose = api.get_fk(ee_link, [0.491, 0.708, -1.878, 1.158, -0.067])
        api.movep_rel(place_pose, 0, 0, 0.1, 0, 0, 0, wait=True)
        api.movel(place_pose, wait=True)
        api.set_digital_io(1, 0)
        rospy.sleep(0.2)
        api.movel_rel(place_pose, 0, 0, 0.1, 0, 0, 0, wait=True)

        # ready
        api.movej(ready_positions, wait=True)

        # back home
        ready_positions[0] = 0.0
        api.movej(ready_positions, wait=True)
        api.movep_rel(home_pose, 0, 0, 0.1, 0, 0, 0, wait=True)
        api.movej([0, 0, 0, 0, 0], wait=True)

    except Exception as e:
        rospy.logerr(e)
    finally:
        api.set_digital_io(1, 0)
        # api.disable(8)
    rospy.loginfo('teardown')

if __name__ == '__main__':
    args = parse_arguments()
    main(args)
