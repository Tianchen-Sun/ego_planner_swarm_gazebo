#ifndef _PICK_FSM_NODE_H_
#define _PICK_FSM_NODE_H_
#endif
#include <stdlib.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cmath>
#include "std_msgs/Bool.h"

#include <iostream>
#include <termios.h>

class PickFsmNode
{
public: 
    PickFsmNode(const ros::NodeHandle & nh);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void detected_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_check_callback(const ros::TimerEvent& event);
    void publish_selected_goal();
    void set_hover_pose();
    void ask_user_input();
    void publish_yolo_state_signal();
    static char getch();
   

private:
    // subscriber 
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber detected_goal_sub_;

    // timer 
    ros::Timer state_check_timer_;

    // publish the current goal
    ros::Publisher selected_goal_pub_;  
    ros::Publisher yolo_signal_;

    // drone state variables
    Eigen::Vector3d odom_pos_; // odometry state
    Eigen::Quaterniond odom_orient_; // odometry state
    Eigen::Quaterniond detected_goal_orient_; // goal state
    Eigen::Quaterniond hover_orient_; // hover state

    nav_msgs::Odometry odom_pose_; // odometry pose
    geometry_msgs::PoseStamped detected_goal_; // detected goal pose
    geometry_msgs::PoseStamped selected_goal_; // selected goal pose

    // fsm variables
    bool goal_detected_; // whether the goal is detected
    bool at_goal_pose_; // whether the pick is finished
    bool at_goal_position_; // whether the drone is at the goal position

    bool at_hover_pose_; // whether the drone is at the hover pose
    bool set_yolo_state_; // whether the yolo is running
    bool on_the_way_; // whether the drone is on the way to the goal pose
    std_msgs::Bool yolo_state_signal_; // yolo state signal

    // set constant Hover position for apple detection
    geometry_msgs::PoseStamped hover_pose_; 

    // set constant
    double goal_threshold_;
    double orient_tolerance_ = 1e-6;
};