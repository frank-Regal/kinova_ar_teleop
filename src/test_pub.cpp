/* 
*  ROS node to publish a simple series of geometry_msgs/PoseStamped 
*  in the z-direction for testing and debugging teleop_w_twist node 
*  and <one/two>_hand_teleop_test.launch files.
*  
*  Updated: July 2022
*  Author: Frank Regal
*  Email: fregal@utexas.edu
*  
*  See Github README for usage and development help.
*/

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pub");
    ros::NodeHandle n;
    geometry_msgs::PoseStamped msg;
    std::string hand_pose_sub_topic;
    std::string hand;
    hand = "left";
    std::string ns = ros::this_node::getNamespace();
    //ros::param::get("/" + ns + "/hand_pose_sub_topic", hand_pose_sub_topic);

    ros::Publisher geo_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/controller", 1000);

    float rate = 5;
    ros::Rate loop_rate(rate);

    double cur_x {0.162942};
    double cur_y {-0.0386004};
    double cur_z {0.610716};
    msg.pose.position.z = cur_z;
    double increments {0.01};
    int num_increments {20};
    int i {0};

    while (ros::ok())
    {
        msg.pose.position.x = cur_x;
        msg.pose.position.y = cur_y;
        msg.pose.position.z = msg.pose.position.z + increments;
        msg.pose.orientation.x = 0.5154261882691636;
        msg.pose.orientation.y = -0.49582307219529964;
        msg.pose.orientation.z = 0.5075196946022041;
        msg.pose.orientation.w = -0.4805404094517134;
        i = i + 1;
        if (i > num_increments){
            msg.pose.position.z = cur_z;
            i = 0;
        }
        geo_pose_pub.publish(msg);
        std::cout << "[" << hand_pose_sub_topic << "] published at " << rate << " Hz" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}