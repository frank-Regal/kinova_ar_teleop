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
    start_string ("[ALL SYSTEMS GO] Servoing Kinova with AR Hands ..."),
    prev_left_time (0),
    prev_right_time (0),
    prev_time (0)
{

}

HololensUtility::~HololensUtility()
{
    
}

void HololensUtility::GetStartUpMsg()
{
    std::cout << start_string << std::endl;
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

    // init
    double delta_t = 0;

    // calculate
    delta_t = cur_time - prev_time;

    // Convert
    out_twist.lin_x = trans_scale * ((cur_pose.x - robot_pose.x) / delta_t);
    out_twist.lin_y = trans_scale * ((cur_pose.y - robot_pose.y) / delta_t);
    out_twist.lin_z = trans_scale * ((cur_pose.z - robot_pose.z) / delta_t);
    
    /* Don't move orientation */
    out_twist.ang_x = 0;
    out_twist.ang_y = 0;
    out_twist.ang_z = 0;

    /* Used to Calculate actual angular twist (NOT OPTIMIZED)
    out_twist.ang_x = rot_scale * ((cur_pose.theta_x - robot_pose.theta_x) / 1);
    out_twist.ang_y = rot_scale * ((cur_pose.theta_y - robot_pose.theta_y) / 1);
    out_twist.ang_z = rot_scale * ((cur_pose.theta_z - robot_pose.theta_z) / 1);
    */

    //prev_pose = cur_pose;
    prev_time = cur_time;

}