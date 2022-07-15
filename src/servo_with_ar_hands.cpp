#include <iostream>
#include <string>

#include "ros/ros.h"
#include "kinova_ar_teleop/hololens_utility.h"
#include "geometry_msgs/PoseStamped.h"
#include "kortex_driver/TwistCommand.h"

HololensUtility HololensUtil;
ros::Publisher kortex_twist_pub_;
ros::Subscriber left_hand_pose_;
ros::Subscriber right_hand_pose_;

void publish_twist(const ros::TimerEvent&)
{
   kortex_driver::TwistCommand ctrl;
   ctrl.twist.linear_x = 0.0;
   ctrl.twist.linear_y = 0.0;
   ctrl.twist.linear_z = 0.0;
   ctrl.twist.angular_x = 0.0;
   ctrl.twist.angular_y = 0.0;
   ctrl.twist.angular_z = 0.0;
   kortex_twist_pub_.publish(ctrl);
}

void left_hand_callback(const geometry_msgs::PoseStamped msg)
{
    std::cout << "Left Hand Message Received" << std::endl;
}

void right_hand_callback(const geometry_msgs::PoseStamped msg)
{
    std::cout << "Right Hand Message Received" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"servo_with_ar_hands");
    ros::NodeHandle n;

    // Debug
    HololensUtil.GetTestString();
    
    left_hand_pose_ = n.subscribe<geometry_msgs::PoseStamped>("/left/controller", 1000, left_hand_callback);
    right_hand_pose_ = n.subscribe<geometry_msgs::PoseStamped>("/right/controller", 1000, right_hand_callback);
    
    kortex_twist_pub_ = n.advertise<kortex_driver::TwistCommand>("/my_gen3/in/cartesian_velocity",1000);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), publish_twist);

    ros::spin();
    
    return 0;
}
