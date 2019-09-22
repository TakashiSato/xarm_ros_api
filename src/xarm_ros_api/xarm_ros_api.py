# -*- coding: utf-8 -*-
import rospy
import json
import tf
import moveit_commander
import utils

from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from xarm_planner.srv import joint_plan, pose_plan, single_straight_plan, exec_plan
from xarm_msgs.srv import SetDigitalIO, ClearErr, SetAxis, SetInt16

from shape_msgs.msg import SolidPrimitive

class XArmRosApi():

    def __init__(self, move_group, ee_link, sim):
        self._sim = sim
        self._ee_link = ee_link
        self.__create_publishers()
        self.__create_services(sim)
        self._arm = utils.get_move_group_commander(move_group)
        self._active_joitns = self._arm.get_active_joints()

    def __create_publishers(self):
        self._pubs = {}

        def create_publihser(pubs, topic_name, msg_type, queue_size=1):
            pub = rospy.Publisher(topic_name, msg_type, queue_size=queue_size)
            pubs[topic_name] = pub

        publisher_dict = {
            'xarm_planner_exec': Bool,
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
            'xarm_exec_plan' : exec_plan,
            'compute_fk' :GetPositionFK,
        }

        if not sim:
            real_hw_service_dict = {
                'xarm/clear_err' : ClearErr,
                'xarm/set_digital_out' : SetDigitalIO,
                'xarm/motion_ctrl' : SetAxis,
                'xarm/set_mode' : SetInt16,
                'xarm/set_state' : SetInt16,
            }
            service_dict.update(real_hw_service_dict)

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

    def __call_real_hw_service(self, service_name, srv_args):
        if self._sim:
            return True
        response = self.__call_service(service_name, srv_args)
        return (response.ret == 0)

    def enable(self, id):
        return self.__call_real_hw_service('xarm/motion_ctrl', {'id': id, 'data': 1})

    def disable(self, id):
        return self.__call_real_hw_service('xarm/motion_ctrl', {'id': id, 'data': 0})

    def clear_error(self):
        return self.__call_real_hw_service('xarm/clear_err', {})

    def set_mode(self, data):
        return self.__call_real_hw_service('xarm/set_mode', {'data': data})

    def set_state(self, data):
        return self.__call_real_hw_service('xarm/set_state', {'data': data})

    def set_digital_io(self, io_num, value):
        return self.__call_real_hw_service('xarm/set_digital_out', {'io_num': io_num, 'value': value})

    def get_current_joint_positions(self):
        return self._arm.get_current_joint_values()

    def get_current_pose(self):
        return self._arm.get_current_pose().pose

    def get_fk(self, tooltip, positions):
        header = Header(0, rospy.Time.now(), 'world')
        robot_state = RobotState()
        robot_state.joint_state.name = self._active_joitns
        robot_state.joint_state.position = positions
        response = self.__call_service('compute_fk', {'header': header, 'fk_link_names': [tooltip], 'robot_state': robot_state})
        if len(response.pose_stamped) == 0:
            return None
        return response.pose_stamped[0].pose

    def create_pose_from_quaternion(self, x, y, z, qx, qy, qz, qw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def create_pose_from_euler(self, x, y, z, roll, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return self.create_pose_from_quaternion(x, y, z, q[0], q[1], q[2], q[3])

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
        return self.exec_plan(wait=wait) if self.plan_movej(positions) else False

    def movep(self, pose, wait=True):
        return self.exec_plan(wait=wait) if self.plan_pose(pose) else False

    def movel(self, pose, wait=True):
        return self.exec_plan(wait=wait) if self.plan_straight(pose) else False

    def movej_rel(self, positions, dpositions, wait=True):
        return self.exec_plan(wait=wait) if self.plan_movej(positions+dpositions) else False

    def movep_rel(self, pose, dx, dy, dz, droll, dpitch, dyaw, wait=True):
        pose = utils.create_add_diff_pose(pose, dx, dy, dz, droll, dpitch, dyaw)
        return self.movep(pose)

    def movel_rel(self, pose, dx, dy, dz, droll, dpitch, dyaw, wait=True):
        pose = utils.create_add_diff_pose(pose, dx, dy, dz, droll, dpitch, dyaw)
        return self.movel(pose)
