#include <plan_manage/pick_fsm_node.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PickFsmNode::PickFsmNode(const ros::NodeHandle & nh){
    nh_=nh; 

    nh_.param<double>("goal_threshold", goal_threshold_,0.5f);
    nh_.param<double>("orient_tolerance", orient_tolerance_, 1e-1f);
    nh_.param<bool>("test_mode", test_, false);
    on_the_way_ = false;
    ROS_INFO("goal_threshold = %f", goal_threshold_);
    ROS_INFO("orient_tolerance = %f", orient_tolerance_);


    goal_detected_ = false;
    odom_sub_ = nh_.subscribe("/odom", 1, &PickFsmNode::odom_callback, this);
    detected_goal_sub_ = nh_.subscribe("/detected_goal_pos", 1, &PickFsmNode::detected_goal_callback, this);
    
    selected_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/selected_goal", 1);
    yolo_signal_ = nh_.advertise<std_msgs::Bool>("/yolo_state_signal", 1);

    state_check_timer_ = nh.createTimer(ros::Duration(1.0),&PickFsmNode::state_check_callback,this);
    
    set_hover_pose();

} 

// justify whether the pick is finished
void PickFsmNode::state_check_callback(const ros::TimerEvent& event){

    // preset the state
    at_goal_pose_ = false;
    at_goal_position_ = false;
    at_hover_pose_ = false;
    
    

    //TEMP
    if (test_==true){
        detected_goal_.pose.position.z = 1.0;
    }

    // check whether the drone is at the goal pose
    if (sqrt(pow(odom_pos_(0)-detected_goal_.pose.position.x,2))+sqrt(pow(odom_pos_(1)-detected_goal_.pose.position.y,2))+sqrt(pow(odom_pos_(2)-detected_goal_.pose.position.z,2))<goal_threshold_)
    {   
        if (odom_orient_.isApprox(detected_goal_orient_, orient_tolerance_)){
            at_goal_pose_ = true;
        }
        else{
            at_goal_pose_ = false;
        }
        at_goal_position_ = true;
    }
    else{
        at_goal_pose_ = false;
    }
    // ROS_INFO("detected_goal_.pose.position.x = %f", detected_goal_.pose.position.x);
    // ROS_INFO("detected_goal_.pose.position.y = %f", detected_goal_.pose.position.y);
    // ROS_INFO("detected_goal_.pose.position.z = %f", detected_goal_.pose.position.z);
    // ROS_INFO("odom_pos_(0) = %f", odom_pos_(0));
    // ROS_INFO("odom_pos_(1) = %f", odom_pos_(1));
    // ROS_INFO("odom_pos_(2) = %f", odom_pos_(2));
    // ROS_INFO("state_check_callback: at_goal_position_ = %d", at_goal_position_);
    // ROS_INFO("state_check_callback: at_goal_pose = %d", at_goal_pose_);


    
    // check whether the drone is at the hover pose

    //TEMP
    if (test_==true){
        odom_pos_(2) = hover_pose_.pose.position.z;
    }
    at_hover_position_ = false;
    if (sqrt(pow(odom_pos_(0)-hover_pose_.pose.position.x,2))+sqrt(pow(odom_pos_(1)-hover_pose_.pose.position.y,2))+sqrt(pow(odom_pos_(2)-hover_pose_.pose.position.z,2))<goal_threshold_)
    {    
        if (odom_orient_.isApprox(hover_orient_, orient_tolerance_)){
            at_hover_pose_ = true;
        }
        else{
            at_hover_pose_ = false;
        }
        at_hover_position_ =true;
    }
    else{
        at_hover_pose_ = false;
    }
    // ROS_INFO("state_check_callback: at_hover_position = %d", at_hover_position);
    // ROS_INFO("state_check_callback: at_hover_pose = %d", at_hover_pose_);




}

void PickFsmNode::state_transition()
{ 
    //--------- state transition ---------
    // publish the current goal

    /*
    * at_hover_pose_ 
    * on_the_way_
    * at_goal_pose_
    */

   // TEMP：
   // at_hover_pose_ = true;

    if (at_hover_position_==true){
        on_the_way_ = false;
        ROS_INFO("I am hovering!");

        //TEMP
        // ask_user_yolo_input(); //will set set_yolo_state_
        set_yolo_state_ =true;

        if (set_yolo_state_==true){

            // publish the yolo signal to start yolo
            // will publish the yolo signal in the end of the callback function
            ROS_INFO("start yolo signal is published!");

            // subscribe the detected goal
            detected_goal_sub_ = nh_.subscribe("/detected_goal_pos", 1, &PickFsmNode::detected_goal_callback, this);
            ROS_INFO("detected_goal_sub_ is subscribed!");
        }
        else{
            // publish the yolo signal to stop yolo
            // will publish the yolo signal in the end of the callback function
            ROS_INFO("yolo is not running!");
        }
    }
    else if (goal_detected_==true && at_goal_pose_==false){
        execute_flag_=false;
        ask_user_execute_input();
        
        if (execute_flag_)
        {
            publish_selected_goal();
            ROS_INFO("I have found an apple, I will go there! Selected goal published!");

            //Once goal detected, ignore the following detected goal, until current pick is finished
            detected_goal_sub_.shutdown();
        }
        else{
            ROS_INFO("Alright, I will not go there!");
        }
        goal_detected_ = false;
    }
    // else if (on_the_way_==true){
    //     ROS_INFO("I am on the way!");
    //     // publish_selected_goal();
    // }
    // return to the hover position
    
    else if (at_goal_position_==true){
        on_the_way_ = false;
        ROS_INFO("Pick finished!, I will return to hover pose in 5s later and listening targets!");
        
        ros::Duration(5.0).sleep();
        selected_goal_pub_.publish(hover_pose_);
        
    }   
    else if (goal_detected_==false){
        on_the_way_ = false;
        if (at_hover_pose_==false){
            ros::Duration(5.0).sleep();
            selected_goal_pub_.publish(hover_pose_);
            ROS_INFO("I have not found an apple, I will return to hover pose!");
        }
        else if(at_hover_pose_==true){
            ROS_INFO("I have not found an apple, I will hover!");
        }
    }
    else{
        ROS_INFO("Waiting for goal detection!");
    }
    

    // publish the yolo state signal, only if the yolo state is changed
    if (set_yolo_state_!=yolo_state_prev_){
        publish_yolo_state_signal();
        yolo_state_prev_ = set_yolo_state_;
    }
}

// get the odometry state
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
    ROS_INFO("deteced_goal_callback is activated! and goal_detected_ = %d", goal_detected_);
    ROS_INFO("deteced_goal_callback: goal_msg->pose.position.x = %f", msg->pose.position.x);
    ROS_INFO("deteced_goal_callback: goal_msg->pose.position.y = %f", msg->pose.position.y);
    ROS_INFO("deteced_goal_callback: goal_msg-->pose.position.z = %f", msg->pose.position.z);
    detected_goal_.header = msg->header;
    detected_goal_.pose= msg->pose;
    
    detected_goal_orient_.x() = detected_goal_.pose.orientation.x;
    detected_goal_orient_.y() = detected_goal_.pose.orientation.y;
    detected_goal_orient_.z() = detected_goal_.pose.orientation.z;
    detected_goal_orient_.w() = detected_goal_.pose.orientation.w;

}


// publish the selected goal
void PickFsmNode::publish_selected_goal(){

    // publish the selected goal
    selected_goal_.header=detected_goal_.header;
    selected_goal_.pose=detected_goal_.pose;
    selected_goal_pub_.publish(selected_goal_);
}

void PickFsmNode::set_hover_pose(){

    // for publishing
    hover_pose_.header.stamp = ros::Time::now();
    hover_pose_.header.frame_id = "map";

    //TEMP
    hover_pose_.pose.position.x = 0.0;
    hover_pose_.pose.position.y = 0.0;
    if (test_==true){
        hover_pose_.pose.position.x = 9.0;
        hover_pose_.pose.position.y = 2.0;
    }
    
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

// char PickFsmNode::getch() 
// {
//     struct termios oldt, newt;
//     char ch;
//     tcgetattr(STDIN_FILENO, &oldt);
//     newt = oldt;
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//     ch = getchar();
//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//     return ch;
// }

void PickFsmNode::ask_user_yolo_input(){
    std::cout<<"Do you want to run yolo? (y/n)"<<std::endl;
    char user_key=getchar();

    switch(user_key){
        case 'y':
            set_yolo_state_ = true;
            break;
        case 'n':
            set_yolo_state_= false;
            break;
        default:
            break;
    
    }

}
void PickFsmNode::ask_user_execute_input(){

    std::cout << "I have found apple at [" << detected_goal_.pose.position.x
          << "," << detected_goal_.pose.position.y
          << "," << detected_goal_.pose.position.z
          << "]. Do you want me to go there? (y/n)" << std::endl;
    char user_key=getchar();

    switch(user_key){
        case 'y':
            execute_flag_ = true;
            break;
        case 'n':
            execute_flag_= false;
            break;
        default:
            break;
    
    }

}

void PickFsmNode::publish_yolo_state_signal(){
    yolo_state_signal_.data = set_yolo_state_;
    yolo_signal_.publish( yolo_state_signal_);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pick_fsm_node");
  ros::NodeHandle nh("~");
  PickFsmNode pick_fsm_node(nh);


  while (ros::ok()) {
    pick_fsm_node.state_transition();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return 0;
}