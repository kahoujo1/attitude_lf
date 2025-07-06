export ROS_MASTER_URI=http://uav36:11311
rosservice call /uav36/control_manager/goto "goal:
- -50.0
- 10.0
- 5.0
- 0.0"
export ROS_MASTER_URI=http://uav40:11311
rosservice call /uav40/control_manager/goto "goal:
- -40.0
- 10.0
- 5.0
- 0.0"