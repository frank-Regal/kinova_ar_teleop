#!/usr/bin/env python

import rospy
import sys
import os

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import Header
from std_msgs.msg import String

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2




def main():
    pub = rospy.Publisher('test', String, queue_size=10)
    rospy.init_node('hand_control',anonymous=True)
    rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str="hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__=='__main__':
    main()