#include <plan_manage/pick_fsm_node.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PickFsmNode::PickFsmNode(const ros::NodeHandle & nh){
    nh_=nh; 

    nh_.param<double>("goal_threshold", goal_threshold_,0.5f);
    nh_.param<double>("orient_tolerance", orient_tolerance_, 1e-1f);

    ROS_INFO("goal_threshold = %f", goal_threshold_);
    ROS_INFO("orient_tolerance = %f", orient_tolerance_);

    pick_finished_ = false;
    at_hover_pose_ = false;
    goal_detected_ = false;
    odom_sub_ = nh_.subscribe("/odom", 1, &PickFsmNode::odom_callback, this);
    detected_goal_sub_ = nh_.subscribe("/detected_goal_pos", 1, &PickFsmNode::detected_goal_callback, this);
    selected_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/selected_goal", 1);
    
    state_check_timer_ = nh.createTimer(ros::Duration(1.0),&PickFsmNode::state_check_callback,this);
    
    set_hover_pose();
    
    // publish the current goal
    if (goal_detected_==true && pick_finished_==false){
        publish_selected_goal();
        ROS_INFO("I have found an apple, I will go there! Selected goal published!");
    }
    // return to the hover position
    else if (pick_finished_==true){
        ROS_INFO("Pick finished!, I will return to hover pose and listening targets!");
        selected_goal_pub_.publish(hover_pose_);
    }   
    else if (goal_detected_==false){
        ROS_INFO("I have not found an apple, I will hover!");
    }
    else{
        ROS_INFO("Waiting for goal detection!");
    }
} 

// justify whether the pick is finished
void PickFsmNode::state_check_callback(const ros::TimerEvent& event){
    detected_goal_.pose.position.z = 2.0;
    bool at_goal_position = false;
    // check whether the drone is at the goal pose
    if (sqrt(pow(odom_pos_(0)-detected_goal_.pose.position.x,2))+sqrt(pow(odom_pos_(1)-detected_goal_.pose.position.y,2))+sqrt(pow(odom_pos_(2)-detected_goal_.pose.position.z,2))<goal_threshold_)
    {   
        if (odom_orient_.isApprox(detected_goal_orient_, orient_tolerance_)){
            pick_finished_ = true;
        }
        else{
            pick_finished_ = false;
        }
        at_goal_position = true;
    }
    else{
        pick_finished_ = false;
    }

    ROS_INFO("state_check_callback: at_pick_position = %d", at_goal_position);
    ROS_INFO("state_check_callback: at_pick_pose = %d", pick_finished_);

    // check whether the drone is at the hover pose
    odom_pos_(2) = hover_pose_.pose.position.z;
    bool at_hover_position = false;
    if (sqrt(pow(odom_pos_(0)-hover_pose_.pose.position.x,2))+sqrt(pow(odom_pos_(1)-hover_pose_.pose.position.y,2))+sqrt(pow(odom_pos_(2)-hover_pose_.pose.position.z,2))<goal_threshold_)
    {    
        if (odom_orient_.isApprox(hover_orient_, orient_tolerance_)){
            at_hover_pose_ = true;
        }
        else{
            at_hover_pose_ = false;
        }
        at_hover_position =true;
    }
    else{
        at_hover_pose_ = false;
    }
    ROS_INFO("state_check_callback: at_hover_position = %d", at_hover_position);
    ROS_INFO("state_check_callback: at_hover_pose = %d", at_hover_pose_);

    
}

void PickFsmNode::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    odom_pose_.header = msg->header;
    odom_pose_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_pose_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_pose_.pose.pose.position.z = msg->pose.pose.position.z;

    odom_pos_(0) = odom_pose_.pose.pose.position.x;
    odom_pos_(1) = odom_pose_.pose.pose.position.y;
    odom_pos_(2) = odom_pose_.pose.pose.position.z;
    
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    odom_orient_.w() = msg->pose.pose.orientation.w;
    // }
}


void PickFsmNode::detected_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_detected_ = true;
    
    detected_goal_.header = msg->header;
    detected_goal_.pose= msg->pose;
    
    detected_goal_orient_.x() = detected_goal_.pose.orientation.x;
    detected_goal_orient_.y() = detected_goal_.pose.orientation.y;
    detected_goal_orient_.z() = detected_goal_.pose.orientation.z;
    detected_goal_orient_.w() = detected_goal_.pose.orientation.w;

    //Once goal detected, ignore the following detected goal, until current pick is finished
    detected_goal_sub_.shutdown();
}


// publish the selected goal
void PickFsmNode::publish_selected_goal(){

    // publish the selected goal
    selected_goal_.header=detected_goal_.header;
    selected_goal_.pose=detected_goal_.pose;
    
    

}




void PickFsmNode::set_hover_pose(){

    // for publishing
    hover_pose_.header.stamp = ros::Time::now();
    hover_pose_.header.frame_id = "world";
    hover_pose_.pose.position.x = 0.0;
    hover_pose_.pose.position.y = 0.0;
    hover_pose_.pose.position.z = 2.0;
    hover_pose_.pose.orientation.x = 0.0;
    hover_pose_.pose.orientation.y = 0.0;
    hover_pose_.pose.orientation.z = 0.0;
    hover_pose_.pose.orientation.w = 1.0;

    // for orientation comparison
    hover_orient_.x() = hover_pose_.pose.orientation.x;
    hover_orient_.y() = hover_pose_.pose.orientation.y;
    hover_orient_.z() = hover_pose_.pose.orientation.z;
    hover_orient_.w() = hover_pose_.pose.orientation.w;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pick_fsm_node");
  ros::NodeHandle nh("~");

  
  PickFsmNode pick_fsm_node(nh);

  ros::spin();

  return 0;
}