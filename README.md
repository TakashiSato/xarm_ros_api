# xarm_ros_api

## How to use

1. Launch xarm

- Simulation

```
roslaunch xarm_planner xarm_planner_rviz_sim.launch robot_dof:=<7|6|5>
```

- Real HW

```
roslaunch xarm_planner xarm_planner_realHW.launch robot:=<your controller box LAN IP address> robot_dof:=<7|6|5>
```

2. run xarm_ros_api_test.py

```
rosrun xarm_ros_api xarm5_ros_api_test.py
```
