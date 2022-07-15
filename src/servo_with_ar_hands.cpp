#include <iostream>
#include <string>

#include "ros/ros.h"
#include "kinova_ar_teleop/hololens_utility.h"
#include "geometry_msgs/PoseStamped.h"
#include "kortex_driver/TwistCommand.h"

void callback(const ros::TimerEvent&)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"servo_with_ar_hands");
    ros::NodeHandle n;

    HololensUtility test;
    
    test.GetTestString();

    //ros::Publisher kortex_TwistCommand = n.advertise<kortex_driver::TwistCommand>("/my_gen3/in/cartesian_velocity",1000);
    //ros::Subscriber hololens_left_PoseStamped = n.subscribe<geometry_msgs::PoseStamped>("/left/controller", 1000, convert_to_twist);

    //ros::Timer timer = n.createTimer(ros::Duration(0.1), convert_to_twist);

    //ros::spin();
    
    return 0;
}
