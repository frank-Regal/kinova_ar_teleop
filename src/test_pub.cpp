#include <iostream>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_pub");
    ros::NodeHandle n;
    geometry_msgs::PoseStamped msg;

    ros::Publisher geo_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/left/controller", 1000);

    ros::Rate loop_rate(1);

    double cur_x {0.213903};
    double cur_y {-0.0216896};
    double cur_z {0.619689404964};
    msg.pose.position.z = cur_z;
    double increments {0.01};
    int num_increments {20};
    int i {0};

    while (ros::ok())
    {
        msg.pose.position.x = cur_x;
        msg.pose.position.y = cur_y;
        msg.pose.position.z = msg.pose.position.z + increments;
        msg.pose.orientation.x = -0.584;
        msg.pose.orientation.y = 0.464;
        msg.pose.orientation.z = -0.386;
        msg.pose.orientation.w = 0.542;
        i = i + 1;
        if (i > num_increments){
            msg.pose.position.z = cur_z;
            i = 0;
        }
        geo_pose_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    

    return 0;
}