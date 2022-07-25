#include <iostream>
#include <string>
#include <numeric>
#include <limits>

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
ros::Publisher twist_pub_;
ros::Subscriber hand_pose_sub_;
ros::ServiceClient cur_cartesian_pose_;
kortex_driver::GetMeasuredCartesianPose srv_;
kortex_driver::TwistCommand left_twist_;
std::vector<double> msg_rates_;
std::string hand_pose_sub_topic_;
std::string kortex_pose_service_topic_;
std::string kortex_twist_pub_topic_;
double null_twist_pub_rate_;
double cur_time_stamp_;
double msg_time_stamp_;
double prev_time_stamp_;
double variance_;
double pub_rate_;
double rate_;
bool is_first_pub_rate_;
bool is_pose_received_;
bool is_first_msg_;

/*
* grab the current estimated position of the robotic arm
*/
CartesianPose get_current_pose()
{
    // init
    CartesianPose cur_pose;
    tf2::Quaternion q;
    srv_.request.input = {};
    
    // call to Kinova's /<robot_name>/base/get_measured_cartesian_pose sevice
    if (cur_cartesian_pose_.call(srv_))
    {
        // postion
        cur_pose.x = srv_.response.output.x; // [m]
        cur_pose.y = srv_.response.output.y; // [m]
        cur_pose.z = srv_.response.output.z; // [m]
        
        // orientation
        cur_pose.theta_x = srv_.response.output.theta_x * (M_PI/180); // [rad/s]
        cur_pose.theta_y = srv_.response.output.theta_y * (M_PI/180); // [rad/s]
        cur_pose.theta_z = srv_.response.output.theta_z * (M_PI/180); // [rad/s]
        
        // create quaternion to get orientation to match input pose orientation
        q.setRPY(cur_pose.theta_x,cur_pose.theta_y,cur_pose.theta_z); 
        q = q.normalize();

        // convert back to euler
        tf2::getEulerYPR(q,cur_pose.theta_z,cur_pose.theta_y,cur_pose.theta_x);
    }
    else 
    {
        ROS_ERROR("Failed to call service add_two_ints");
        //ros::shutdown();
    }

    return cur_pose;
}

/*
* Pub & Sub Callbacks
*/
void publish_null_twist(const ros::TimerEvent&)
{
    double delta_t = ros::Time::now().toSec() - msg_time_stamp_;
    
    if (!is_pose_received_ && delta_t > (pub_rate_ + variance_))
    {
        // fill
        kortex_driver::TwistCommand twist_msg;
        twist_msg.twist.linear_x = 0;
        twist_msg.twist.linear_y = 0;
        twist_msg.twist.linear_z = 0;
        twist_msg.twist.angular_x = 0;
        twist_msg.twist.angular_y = 0;
        twist_msg.twist.angular_z = 0;

        // publish
        twist_pub_.publish(twist_msg);
        std::cout << "[Twist Published] Sent null" << std::endl;
    }
}

void get_sub_rate(const double& msg_time_stamp)
{   
    // check elapsed time
    double delta_t = msg_time_stamp - prev_time_stamp_;
    
    // skip first subscribed message
    if(!is_first_msg_)
    {
        // assignment for first time in if statement
        if (is_first_pub_rate_)
        {
            pub_rate_ = delta_t;
            is_first_pub_rate_ = false;
        }

        // check for multiple start up and shut downs
        if (delta_t > (pub_rate_ - variance_) && delta_t < (pub_rate_ + variance_))
        {   
            // average pub rates
            msg_rates_.push_back(delta_t);
            pub_rate_ = std::accumulate(msg_rates_.begin(),msg_rates_.end(),0.0) / msg_rates_.size();
        }
    }
    prev_time_stamp_ = msg_time_stamp;
    is_first_msg_ = false;
}

void hand_pose_callback(const geometry_msgs::PoseStamped msg)
{   
    // set global flag
    is_pose_received_ = true;
    msg_time_stamp_ = ros::Time::now().toSec();
    get_sub_rate(msg_time_stamp_);

    // init
    CartesianPose hand_pose;
    CartesianPose robot_pose;
    TwistMsg hand_twist;
    kortex_driver::TwistCommand left_twist;
    
    // fill
    hand_pose.x = msg.pose.position.x;
    hand_pose.y = msg.pose.position.y;
    hand_pose.z = msg.pose.position.z;
    tf2::getEulerYPR(msg.pose.orientation, hand_pose.theta_z, hand_pose.theta_y, hand_pose.theta_x);

    // call
    robot_pose = get_current_pose();

    // convert
    HololensUtil.PoseToTwist(hand_pose,robot_pose, "left", msg_time_stamp_, hand_twist, 1, 1);

    // fill
    left_twist.twist.linear_x = hand_twist.lin_x;
    left_twist.twist.linear_y = hand_twist.lin_y;
    left_twist.twist.linear_z = hand_twist.lin_z;
    left_twist.twist.angular_x = hand_twist.ang_x;
    left_twist.twist.angular_y = hand_twist.ang_y;
    left_twist.twist.angular_z = hand_twist.ang_z;

    // publish
    twist_pub_.publish(left_twist);
    std::cout << "[Twist Published] Sent filled" << std::endl;

    // set global flag
    is_pose_received_ = false;
}

/*
* Core
*/
int main(int argc, char** argv)
{
    // debug
    HololensUtil.GetStartUpMsg();

    // ros node init
    ros::init(argc,argv,"one_hand_teleop_w_twist");
    ros::NodeHandle n;

    // global variable init
    /* ros one_hand_params set in config/one_hand_params.yaml */
    ros::param::get("/null_twist_pub_rate", null_twist_pub_rate_);
    ros::param::get("/kortex_pose_service_topic", kortex_pose_service_topic_);
    ros::param::get("/kortex_twist_pub_topic", kortex_twist_pub_topic_);
    ros::param::get("/hand_pose_sub_topic", hand_pose_sub_topic_);
    ros::param::get("/variance", variance_);
    pub_rate_ = std::numeric_limits<double>::infinity();
    msg_time_stamp_ = ros::Time::now().toSec();
    is_first_pub_rate_ = true;
    is_pose_received_ = false;
    is_first_msg_ = true;
            
    // pub, sub, & client init 
    cur_cartesian_pose_ = n.serviceClient<kortex_driver::GetMeasuredCartesianPose>(kortex_pose_service_topic_);
    hand_pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>(hand_pose_sub_topic_, 1000, hand_pose_callback);
    twist_pub_ = n.advertise<kortex_driver::TwistCommand>(kortex_twist_pub_topic_, 1000);
    ros::Timer timer = n.createTimer(ros::Duration(null_twist_pub_rate_), publish_null_twist);
    
    // ros typical
    ros::spin();
    return 0;
}
