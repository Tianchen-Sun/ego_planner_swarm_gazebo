#roslaunch px4 fast_racing.launch & sleep 20;
# roslaunch ego_planner single_run_in_gazebo.launch & sleep 5;
roslaunch ego_planner single_run_in_gazebo_prm_corridor.launch & sleep 5;
# unlock the drone
rosrun mavros mavparam set COM_RCL_EXCEPT 4
roslaunch px4ctrl run_prm_node.launch & sleep 5;
roslaunch ego_planner rviz_corridor.launch

# in another terminal, start rqt_reconfigure
# rosrun rqt_reconfigure rqt_reconfigure & sleep 1;

# in another terminal, start YOLO
# roslaunch yolov7_ros yolov7.launch
# rosrun dispimg img_disp.py