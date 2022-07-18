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
    prev_right_time{0}
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
    PrintPose(cartesian_pose);
    if (hand == "left"){
        prev_pose_left = cartesian_pose;
    } 
    else {
        prev_pose_right = cartesian_pose;
    }
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
                                  const std::string& hand,
                                  const double& cur_time,
                                  TwistMsg& out_twist,
                                  double trans_scale,
                                  double rot_scale)
{
    // Init
    CartesianPose prev_pose;
    double delta_t {0};
    int hand_int {0};

    // Check
    if (hand == "left"){
        hand_int = 1;
        prev_pose = prev_pose_left;
        delta_t = cur_time - prev_left_time;
        prev_left_time = cur_time;
    }
    else if (hand == "right"){
        hand_int = 2;
        prev_pose = prev_pose_right;
        delta_t = cur_time - prev_left_time;
        prev_right_time = cur_time;
    }
    else {
        std::cout << "ERROR: Undefined string for the name of hand." << std::endl;
        return;
    }

    std::cout << "Delta t: " << delta_t << std::endl;
    // Convert
    out_twist.lin_x = trans_scale * ((cur_pose.x - prev_pose.x) / delta_t);
    out_twist.lin_y = trans_scale * ((cur_pose.y - prev_pose.y) / delta_t);
    out_twist.lin_z = trans_scale * ((cur_pose.z - prev_pose.z) / delta_t);
    out_twist.ang_x = rot_scale * ((cur_pose.theta_x - prev_pose.theta_x) / delta_t);
    out_twist.ang_y = rot_scale * ((cur_pose.theta_y - prev_pose.theta_y) / delta_t);
    out_twist.ang_z = rot_scale * ((cur_pose.theta_z - prev_pose.theta_z) / delta_t);

    // Assign Old
    if (hand_int == 1){
        prev_pose_left = cur_pose;
    }
    else if (hand_int == 2){
        prev_pose_right = cur_pose;
    }

}