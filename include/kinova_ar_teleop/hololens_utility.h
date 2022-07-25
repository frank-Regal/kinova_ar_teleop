#ifndef _HOLOLENS_UTILITY_H_
#define _HOLOLENS_UTILITY_H_

/* 
* Utility used to control HoloLens 2 with Kinova Gen3 Dual Arm Setup
*
* Author: Frank Regal
* Date: 2022-07-15
*/

#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "kortex_driver/TwistCommand.h"

struct CartesianPose {
    double x;       
    double y;       
    double z;       
    double theta_x; 
    double theta_y; 
    double theta_z; 
};

struct TwistMsg {
    double lin_x;
    double lin_y; 
    double lin_z;
    double ang_x;
    double ang_y;
    double ang_z;
};

class HololensUtility
{

private:
    std::string start_string;
    CartesianPose prev_pose;
    double prev_time;
    CartesianPose prev_pose_right;
    double prev_left_time;
    double prev_right_time;

public:
    // Constructor
    HololensUtility();

    // Destructor
    ~HololensUtility();

    // Debug
    void GetStartUpMsg();
    void PrintPose(const CartesianPose& pose);

    // Save the current pose of the hands
    void SavePose(const CartesianPose& cartesian_pose);

    // Convert a pose to a twist
    void PoseToTwist(const CartesianPose& cur_pose,
                     const CartesianPose& robot_pose,
                     const double& prev_time,
                     TwistMsg& out_twist,
                     double trans_scale,
                     double rot_scale);
};

#endif //_HOLOLENS_UTILITY_H_


