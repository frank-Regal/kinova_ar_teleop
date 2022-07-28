#ifndef _HOLOLENS_UTILITY_H_
#define _HOLOLENS_UTILITY_H_

/* 
* Utility used to convert position: x,y,z & orientation: r,p,y into 
* a twist: linear x,y,z & angular x,y,z messages for servoing the arms.
*
* Author: Frank Regal
* Updated: July 2022
* Email: fregal@utexas.edu
*
*/

#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

// provided through the ros_kortex repo on Kinova's official GitHub
// link: https://github.com/Kinovarobotics/ros_kortex
#include "kortex_driver/TwistCommand.h"

// required input format for goal poses
struct CartesianPose {
    double x;       
    double y;       
    double z;       
    double theta_x; 
    double theta_y; 
    double theta_z; 
};

// required output format for twist msgs
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
    // init
    std::string start_string;
    CartesianPose prev_pose;
    double prev_time;
    CartesianPose prev_pose_right;
    double prev_left_time;
    double prev_right_time;

public:
    // constructor
    HololensUtility();

    // destructor
    ~HololensUtility();

    // debug
    void GetStartUpMsg();
    void PrintPose(const CartesianPose& pose);

    // save the current pose of the hands
    void SavePose(const CartesianPose& cartesian_pose);

    // convert a pose to a twist
    void PoseToTwist(const CartesianPose& cur_pose,
                     const CartesianPose& robot_pose,
                     const double& prev_time,
                     TwistMsg& out_twist,
                     double trans_scale,
                     double rot_scale);
};

#endif //_HOLOLENS_UTILITY_H_


