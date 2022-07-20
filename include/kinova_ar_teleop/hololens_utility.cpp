#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "kinova_ar_teleop/hololens_utility.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/utils.h"

HololensUtility::HololensUtility() : 
    test_string{"[Started] Servoing Kinova with AR Hands ..."},
    prev_left_time{0},
    prev_right_time{0},
    prev_time{0}
{

}

HololensUtility::~HololensUtility()
{
    
}

void HololensUtility::GetTestString()
{
    std::cout << test_string << std::endl;
}

void HololensUtility::SavePose(const CartesianPose& cartesian_pose, const std::string& hand)
{
    //PrintPose(cartesian_pose);
    //if (hand == "left"){
    //    prev_pose_left = cartesian_pose;
    //} 
    //else {
    //    prev_pose_right = cartesian_pose;
    //}
}

void HololensUtility::PrintPose(const CartesianPose& pose)
{
    std::cout << "Current Pose: " << std::endl;
    std::cout << "x: " << pose.x << std::endl;
    std::cout << "y: " << pose.y << std::endl;
    std::cout << "z: " << pose.z << std::endl;
    std::cout << "theta_x: " << pose.theta_x << std::endl;
    std::cout << "theta_y: " << pose.theta_y << std::endl;
    std::cout << "theta_z: " << pose.theta_z << std::endl;
}

void HololensUtility::PoseToTwist(const CartesianPose& cur_pose,
                                  const CartesianPose& robot_pose,
                                  const std::string& hand,
                                  const double& cur_time,
                                  TwistMsg& out_twist,
                                  double trans_scale,
                                  double rot_scale)
{

    // Init
    //CartesianPose prev_pose;
    double delta_t {0};
    //int hand_int {0};
    prev_pose = robot_pose;

    delta_t = cur_time - prev_time;
    std::cout << "delta_t: " << delta_t << std::endl;


    // Convert
    out_twist.lin_x = trans_scale * ((cur_pose.x - prev_pose.x) / delta_t);
    out_twist.lin_y = trans_scale * ((cur_pose.y - prev_pose.y) / delta_t);
    out_twist.lin_z = trans_scale * ((cur_pose.z - prev_pose.z) / delta_t);
    out_twist.ang_x = 0;//rot_scale * ((cur_pose.theta_x - prev_pose.theta_x) / 1);
    out_twist.ang_y = 0;//rot_scale * ((cur_pose.theta_y - prev_pose.theta_y) / 1);
    out_twist.ang_z = 0;//rot_scale * ((cur_pose.theta_z - prev_pose.theta_z) / 1);
   
    //prev_pose = cur_pose;
    prev_time = cur_time;

}