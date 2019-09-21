# -*- coding: utf-8 -*-
import rospy
import json
import tf
import moveit_commander
import utils

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from xarm_planner.srv import joint_plan, pose_plan, single_straight_plan, exec_plan


class XArmRosApi():

    def __init__(self, move_group, sim):
        self.__create_publishers()
        self.__create_services(sim)
        self.__arm = utils.get_move_group_commander(move_group)

    def __create_publishers(self):
        self._pubs = {}

        def create_publihser(pubs, topic_name, msg_type, queue_size=1):
            pub = rospy.Publisher(topic_name, msg_type, queue_size=queue_size)
            pubs[topic_name] = pub

        publisher_dict = {
            'xarm_planner_exec': Bool
        }

        controller_pubs = {}
        for topic_name, msg_type in publisher_dict.items():
            create_publihser(controller_pubs, topic_name, msg_type)
        self._pubs = controller_pubs

    def __publish(self, topic_name, msg):
        if topic_name not in self._pubs:
            raise Exception(
                'XArmRosApi dose not have [%s] publisher with args: ' % topic_name)
        rospy.loginfo('Publish to [%s].' %
                      (rospy.get_namespace() + topic_name))
        rospy.loginfo(msg)
        return self._pubs[topic_name].publish(msg)

    def __create_services(self, sim):
        self._srvs = {}

        def create_service(srvs, service_name, service_type):
            srv = utils.create_service(service_name, service_type)
            if srv != None:
                srvs[service_name] = srv

        service_dict = {
            'xarm_joint_plan' : joint_plan,
            'xarm_pose_plan' : pose_plan,
            'xarm_straight_plan' : single_straight_plan,
            'xarm_exec_plan' : exec_plan
        }

        # if not sim:
        #     real_hw_service_dict = {
        #         'xarm/set_' : exec_plan
        #     }

        controller_srvs = {}
        for service_name, service_type in service_dict.items():
            create_service(controller_srvs, service_name, service_type)
        self._srvs = controller_srvs

    def __call_service(self, service_name, srv_args):
        if service_name not in self._srvs:
            raise Exception(
                'XArmRosApi dose not have [%s] service with args: ' % service_name)
        # rospy.loginfo('Call [%s] service with args %s.' %
        #               ((rospy.get_namespace() + service_name), json.dumps(srv_args)))
        return self._srvs[service_name](**srv_args)

    # def servo_on(self, joint_names):
    #     self.__call_set_joint_names_service('robot_hw/servo_on', joint_names)

    # def servo_off(self, joint_names):
    #     self.__call_set_joint_names_service('robot_hw/servo_off', joint_names)

    # def clear_error(self, joint_names):
    #     self.__call_set_joint_names_service(
    #         'robot_hw/clear_error', joint_names)

    def plan_movej(self, positions):
        result = self.__call_service('xarm_joint_plan', {'target': positions})
        return result.success

    def plan_pose(self, pose):
        result = self.__call_service('xarm_pose_plan', {'target': pose})
        return result.success

    def plan_straight(self, pose):
        result = self.__call_service('xarm_straight_plan', {'target': pose})
        return result.success

    def exec_plan(self, wait=True):
        if wait:
            result = self.__call_service('xarm_exec_plan', {'exec_': True})
            if not result.success:
                return False
        else:
            self.__publish('xarm_planner_exec', Bool(True))
        return True

    def movej(self, positions, wait=True):
        if not self.plan_movej(positions):
            return False
        if not self.exec_plan(wait=wait):
            return False
        return True

    def movep(self, dx, dy, dz, droll, dpitch, dyaw, wait=True):
        cur_pos = self.__arm.get_current_pose().pose.position
        cur_rpy = self.__arm.get_current_rpy()
        pose = utils.create_pose(cur_pos, cur_rpy, dx, dy, dz, droll, dpitch, dyaw)
        self.plan_pose(pose)
        if not self.exec_plan(wait=wait):
            return False
        return True

    def movel(self, dx, dy, dz, droll, dpitch, dyaw, wait=True):
        cur_pos = self.__arm.get_current_pose().pose.position
        cur_rpy = self.__arm.get_current_rpy()
        pose = utils.create_pose(cur_pos, cur_rpy, dx, dy, dz, droll, dpitch, dyaw)
        self.plan_straight(pose)
        if not self.exec_plan(wait=wait):
            return False
        return True
