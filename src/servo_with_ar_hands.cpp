#include <iostream>
#include <string>

#include "ros/ros.h"
#include "kinova_ar_teleop/hololens_utility.h"
#include "geometry_msgs/PoseStamped.h"
#include "kortex_driver/TwistCommand.h"
#include "kortex_driver/GetMeasuredCartesianPose.h"
#include "tf2/convert.h"
#include "tf2/utils.h"

/*
* Init
*/
HololensUtility HololensUtil;
ros::Publisher left_kortex_twist_pub_;
ros::Publisher right_kortex_twist_pub_;
ros::Subscriber left_hand_pose_;
ros::Subscriber right_hand_pose_;
ros::ServiceClient cur_cartesian_pose_;
kortex_driver::GetMeasuredCartesianPose srv_;
kortex_driver::TwistCommand left_twist_;
std::string side;


CartesianPose get_current_pose()
{
    CartesianPose cur_pose;
    tf2::Quaternion q;
    srv_.request.input = {};
    if (cur_cartesian_pose_.call(srv_))
    {
        cur_pose.x = srv_.response.output.x;
        cur_pose.y = srv_.response.output.y;
        cur_pose.z = srv_.response.output.z;
        cur_pose.theta_x = srv_.response.output.theta_x * (M_PI/180);
        cur_pose.theta_y = srv_.response.output.theta_y * (M_PI/180);
        cur_pose.theta_z = srv_.response.output.theta_z * (M_PI/180);
        
        q.setRPY(cur_pose.theta_x,cur_pose.theta_y,cur_pose.theta_z);
        q = q.normalize();
        tf2::getEulerYPR(q,cur_pose.theta_z,cur_pose.theta_y,cur_pose.theta_x);
        //HololensUtil.SavePose(cur_pose, side);
        std::cout << "theta_x: " << cur_pose.theta_x << std::endl;
        std::cout << "theta_y: " << cur_pose.theta_y << std::endl;
        std::cout << "theta_z: " << cur_pose.theta_z << std::endl;
    }
    else 
    {
        ROS_ERROR("Failed to call service add_two_ints");
    }

    std::cout << "returned" << std::endl;
    return cur_pose;
}

/*
* Callbacks
*/
void publish_left_twist(const ros::TimerEvent&)
{

}

void left_hand_callback(const geometry_msgs::PoseStamped msg)
{   
    // Init
    CartesianPose hand_pose;
    CartesianPose robot_pose;
    TwistMsg hand_twist;
    kortex_driver::TwistCommand left_twist;
    
    // Fill
    hand_pose.x = msg.pose.position.x;
    hand_pose.y = msg.pose.position.y;
    hand_pose.z = msg.pose.position.z;
    tf2::getEulerYPR(msg.pose.orientation,hand_pose.theta_z,hand_pose.theta_y,hand_pose.theta_x);

    
    std::cout << "Euler Conversion: " << std::endl;
    std::cout << "Hand theta_x: " << hand_pose.theta_x << std::endl;
    std::cout << "Hand theta_y: " << hand_pose.theta_y << std::endl;
    std::cout << "Hand theta_z: " << hand_pose.theta_z << std::endl;
    robot_pose = get_current_pose();

    // Convert
    HololensUtil.PoseToTwist(hand_pose,robot_pose, "left", ros::Time::now().toSec(), hand_twist, 1, 1);

    // Fill
    left_twist.twist.linear_x = hand_twist.lin_x;
    left_twist.twist.linear_y = hand_twist.lin_y;
    left_twist.twist.linear_z = hand_twist.lin_z;
    left_twist.twist.angular_x = hand_twist.ang_x;
    left_twist.twist.angular_y = hand_twist.ang_y;
    left_twist.twist.angular_z = hand_twist.ang_z;

    // Publish
    left_kortex_twist_pub_.publish(left_twist);
}

void right_hand_callback(const geometry_msgs::PoseStamped msg)
{
    //std::cout << "Right Hand Message Received" << std::endl;
    // Init

    CartesianPose hand_pose;
    TwistMsg hand_twist;
    kortex_driver::TwistCommand right_twist;

    // Fill
    hand_pose.x = msg.pose.position.x;
    hand_pose.y = msg.pose.position.y;
    hand_pose.z = msg.pose.position.z;
    tf2::getEulerYPR(msg.pose.orientation,hand_pose.theta_z,hand_pose.theta_y,hand_pose.theta_x);

    // Convert
    //HololensUtil.PoseToTwist(hand_pose,"right",ros::Time::now().toSec(), hand_twist, 1, 1);
    
    // Fill
    right_twist.twist.linear_x = hand_twist.lin_x;
    right_twist.twist.linear_y = hand_twist.lin_y;
    right_twist.twist.linear_z = hand_twist.lin_z;
    right_twist.twist.angular_x = hand_twist.ang_x;
    right_twist.twist.angular_y = hand_twist.ang_y;
    right_twist.twist.angular_z = hand_twist.ang_z;

    // Publish
    right_kortex_twist_pub_.publish(right_twist);
}

/*
* Core
*/
int main(int argc, char** argv)
{
    // Debug
    HololensUtil.GetTestString();

    ros::init(argc,argv,"servo_with_ar_hands");
    ros::NodeHandle n;

    // specify which hand and robot
    side = "right";

    // Init Pub, Sub, & Client 
    cur_cartesian_pose_ = n.serviceClient<kortex_driver::GetMeasuredCartesianPose>("/my_gen3/base/get_measured_cartesian_pose");
    left_hand_pose_ = n.subscribe<geometry_msgs::PoseStamped>("/left/controller", 1000, left_hand_callback);
    right_hand_pose_ = n.subscribe<geometry_msgs::PoseStamped>("/right/controller", 1000, right_hand_callback);
    left_kortex_twist_pub_ = n.advertise<kortex_driver::TwistCommand>("/my_gen3/in/cartesian_velocity",1000);
    right_kortex_twist_pub_ = n.advertise<kortex_driver::TwistCommand>("/my_gen3/in/cartesian_velocity",1000);
    //ros::Timer timer = n.createTimer(ros::Duration(0.01), publish_left_twist);

    // Get Current Robot Pose
    //get_current_pose();

    ros::spin();
    return 0;
}
