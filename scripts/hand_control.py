#!/usr/bin/env python3

from cmath import pi
from distutils.command.install_egg_info import safe_name
import rospy
import sys
import os
import argparse
#import tf2_ros

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import Header
from std_msgs.msg import String

# Imports from Kortex API.
# Link to Install: https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2

# Import the utilities helper module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities as util_hand


class HandControl:
    
    def __init__(self):
        # Setup Robot Connection
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.61")
        self.parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
        self.parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
        self.robot_args = self.parser.parse_args()
        
    def get_inverse_kinematics(self, position, orientation):
        # Create connection to the device and get the router
        with util_hand.DeviceConnection.createTcpConnection(self.robot_args) as router:
            # Create required services
            self.base = BaseClient(router)
            #self.example_forward_kinematics(self.base)
            self.calc_inverse_kinematics(self.base, position, orientation)
    
    def calc_forward_kinematics(self, base):
        # Current arm's joint angles (in home position)
        try:
            print("Getting Angles for every joint...")
            input_joint_angles = base.GetMeasuredJointAngles()
        except KServerException as ex:
            print("Unable to get joint angles")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        print("Joint ID : Joint Angle")
        for joint_angle in input_joint_angles.joint_angles:
            print(joint_angle.joint_identifier, " : ", joint_angle.value)
        print()

        # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
        try:
            print("Computing Foward Kinematics using joint angles...")
            pose = base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to compute forward kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        print("Pose calculated : ")
        print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
        print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
        print()
        return True

    def calc_inverse_kinematics(self, base, position, orientation):

        ## get robot's pose (by using forward kinematics)
        try:
            input_joint_angles = base.GetMeasuredJointAngles()
            pose = base.ComputeForwardKinematics(input_joint_angles)
            #print("Pose calculated : ")
            #print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
            #print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
            #print()
        except KServerException as ex:
            print("Unable to get current robot pose")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        # Object containing cartesian coordinates and Angle Guess
        input_IkData = Base_pb2.IKData()

        # Fill the IKData Object with the cartesian coordinates that need to be converted
        input_IkData.cartesian_pose.x = position.x
        input_IkData.cartesian_pose.y = position.y
        input_IkData.cartesian_pose.z = position.z
        input_IkData.cartesian_pose.theta_x = pose.theta_x # need to change
        input_IkData.cartesian_pose.theta_y = pose.theta_y # need to change
        input_IkData.cartesian_pose.theta_z = pose.theta_z # need to change

        # Fill the IKData Object with the guessed joint angles
        for joint_angle in input_joint_angles.joint_angles :
            jAngle = input_IkData.guess.joint_angles.add()
            # '- 1' to generate an actual "guess" for current joint angles
            jAngle.value = joint_angle.value - 1

        try:
            print("Computing Inverse Kinematics using joint angles and pose...")
            computed_joint_angles = base.ComputeInverseKinematics(input_IkData) # MAIN CALL TO IK SOLVER
        except KServerException as ex:
            print("Unable to compute inverse kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        print("Joint ID : Joint Angle")
        joint_identifier = 0
        for joint_angle in computed_joint_angles.joint_angles :
            
            # ****** NEED TO CONVERT TO DEGREES AND PUBLISH ON /joint_state ROS TOPIC ****
            joint_angle.value = joint_angle.value * (3.14/180)

            print(joint_identifier, " : ", joint_angle.value)
            joint_identifier += 1
        return True

hand_ctrl = HandControl()

# call to inverse kinematics
def callback(msg):

    # Need to convert incoming Quaternion to Euler Angles
    #q = (msg.pose.orientation.x,
    #     msg.pose.orientation.y,
    #     msg.pose.orientation.z,
    #     msg.pose.orientation.w)
    #euler = tf.transformations.euler_from_quaternion(q)

    hand_ctrl.get_inverse_kinematics(msg.pose.position,msg.pose.orientation)
    print()
    print("inverse kinematics called")


def main():

    rospy.init_node('hand_control',anonymous=True)

    # subscriber
    rospy.Subscriber('/controller', PoseStamped, callback, queue_size=1)    

    # publisher
    pub = rospy.Publisher('test', String, queue_size=10)

    # 
    rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str="hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__=='__main__':
    main()