#!/bin/bash

SESSION="ego_prm_gazebo"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)


#####
# Directories
#####

## get current directory

## SCRIPT_DIR: /home/tianchensun/ego_planner_swarm_gazebo/src/script/..
# SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ego_planner_swarm_gazebo_DIR="$SCRIPT_DIR/../.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../PX4-Autopilot"
# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "ego_planner_swarm_gazebo_DIR: $ego_planner_swarm_gazebo_DIR"
# echo "PX4_AUTOPILOT_REPO_DIR: $PX4_AUTOPILOT_REPO_DIR"
# #####
# # Sourcing
# #####
SOURCE_BASHRC="source ~/.bashrc "
SOURCE_WS="
source $ego_planner_swarm_gazebo_DIR/devel/setup.bash
"

#####
# Commands
#####
CMD_0="roslaunch px4 fast_test_more_objects.launch"

CMD_1="
~/QGroundControl.AppImage
"

CMD_2="
roslaunch ego_planner single_run_in_gazebo.launch

"

CMD_3="rosrun mavros mavparam set COM_RCL_EXCEPT 4"

CMD_4="roslaunch px4ctrl run_node.launch"

CMD_5="roslaunch ego_planner rviz.launch"

CMD_6="sleep 20 ; rosrun rqt_reconfigure rqt_reconfigure"

if [ "$SESSIONEXISTS" = "" ]
then 
    # window 0: PX4; QGC;
    tmux new-session -d -s $SESSION

     
    
    # pane 1: QGC 
    tmux split-window -t $SESSION:0.0 -h # got $SESSION:0.1
    

    # pane 2: ego_planner; mavros; px4ctrl; rviz
    tmux select-pane -t $SESSION:0.0  # SWITCH TO PANE 0
    tmux split-window -t $SESSION:0.0 -v # got $SESSION:0.2
    
    
    

    # pane 3: rqt_reconfigure
    tmux select-pane -t $SESSION:0.2  # SWITCH TO PANE 2
    tmux split-window -t $SESSION:0.2 -v # got $SESSION:0.3

    
    # pane 0: PX4 
    tmux send-keys -t $SESSION:0.0 "$SOURCE_BASH $CMD_0" C-m
    
    # pane 1: QGC 
    tmux send-keys -t $SESSION:0.1 "sleep 10 $SOURCE_WS $CMD_1" C-m 
    

    # pane 2: ego_planner; mavros; px4ctrl; rviz
    tmux send-keys -t $SESSION:0.2 "sleep 1; $SOURCE_BASH $SOURCE_WS"  Enter 

    # sleep is outside the CMD, important
    sleep 2
    tmux send-keys -t $SESSION:0.2 "sleep 3; $CMD_4 &" Enter
    sleep 2
    tmux send-keys -t $SESSION:0.2 "sleep 3; $CMD_5 &" Enter
    sleep 2
    tmux send-keys -t $SESSION:0.2 "sleep 3; $CMD_3 &" Enter
    sleep 2

    # the running of CMD2 will block the execution of the following commands
    # so it is executed in the end
    tmux send-keys -t $SESSION:0.2 "sleep 3; $CMD_2 &" Enter


    # pane 3: rqt_reconfigure
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_6" C-m
    


    fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"




# roslaunch ego_planner single_run_in_gazebo.launch & sleep 5;
# roslaunch ego_planner single_run_in_gazebo_prm_corridor.launch & sleep 5;
# # unlock the drone
# rosrun mavros mavparam set COM_RCL_EXCEPT 4
# roslaunch px4ctrl run_prm_node.launch & sleep 5;
# roslaunch ego_planner rviz_corridor.launch

# # in another terminal, start rqt_reconfigure
# rosrun rqt_reconfigure rqt_reconfigure & sleep 1;

# in another terminal, start YOLO
# roslaunch yolov7_ros yolov7.launch
# rosrun dispimg img_disp.py