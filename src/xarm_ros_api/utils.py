# -*- coding: utf-8 -*-
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import ListControllers

def create_service(service_name, service_type, timeout=3.0):
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        return rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException, e:
        rospy.logerr(
            'Service [%s] is unavailable. %s' % (service_name, e))
    return None

def get_controller_states(controller_manager_ns='controller_manager'):
    srv = create_service(controller_manager_ns + '/list_controllers', ListControllers)
    if srv == None:
        return None
    return srv()


def get_controller_dict(controller_manager_ns='controller_manager', including_null_resources=False):
    controller_states = get_controller_states(controller_manager_ns)
    if controller_states == None:
        return {}
    controller_dict = {}
    for state in controller_states.controller:
        if (state.claimed_resources[0].resources == []) and (including_null_resources == False):
            continue
        controller_dict[state.name] = state.claimed_resources[0].resources
    return controller_dict

def get_move_group_commander(name):
    """
    Function for getting moveit's move_group commander

    Parameters
    ----------
    name : string
        move_group's name

    Returns
    -------
    commander : MoveGroupCommander
        moveit's move_group commander
    """

    # Initializing move_group_commander.
    robot = moveit_commander.RobotCommander(robot_description=rospy.get_namespace() + "robot_description" , ns=rospy.get_namespace())
    commander = moveit_commander.MoveGroupCommander(name, robot_description=rospy.get_namespace() +"robot_description", ns=rospy.get_namespace())
    # Settings.
    commander.set_planner_id('RRTConnectkConfigDefault')
    commander.set_max_velocity_scaling_factor(1.0)
    commander.set_max_acceleration_scaling_factor(1.0)
    commander.set_goal_orientation_tolerance(0.001)
    commander.set_goal_position_tolerance(0.001)
    commander.set_planning_time(5.0)
    commander.set_num_planning_attempts(20)
    return commander

def create_pose(pos, rpy, dx, dy, dz, droll, dpitch, dyaw):
    q = tf.transformations.quaternion_from_euler(rpy[0]+droll, rpy[1]+dpitch, rpy[2]+dyaw)
    pose = Pose()
    pose.position.x = pos.x + dx
    pose.position.y = pos.y + dy
    pose.position.z = pos.z + dz
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose
