#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Jackal:
    # Initialization of variables
    __x, __y, __z = 0.0, 0.0, 0.0
    __theta = 0.0
    __freq = 0
    __rot_mat = np.identity(3, dtype=float)

    def __init__(self, freq):
        # Initialize the ROS node
        rospy.init_node('jackal_controller', anonymous=True)

        self.__freq = freq

        # Wait for the Jackal system to start 
        rospy.sleep(2)
        print("--READY--")

        # Create a Publiser for the Jackal's velocity topic
        self.__vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10) 
        
        # Create a Subscriber for the Jackal's odometry topic
        odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.__getPoseInfo)

        # Create Twist messege for speed comands
        self.__vel_cmd = Twist()
        self.__vel_cmd.linear.x = 0
        self.__vel_cmd.angular.z = 0

        
        rate = rospy.Rate(freq)
         
    def __getPoseInfo(self, data):
        position = data.pose.pose.position

        self.__x = position.x
        self.__y = position.y
        self.__z = position.y

        orientation = data.pose.pose.orientation
        #
        self.__theta = 2 * math.atan2(orientation.z, orientation.w)

        q = [orientation.w, orientation.x, orientation.y, orientation.y]
        #self.__rot_mat = Rotation.from_quat(q).as_matrix()
        self.__rot_mat = self.rotZ(self.__theta)

    def rotZ(self, theta):
        R = np.array([ [math.cos(theta), -math.sin(theta), 0.0],
                       [math.sin(theta),  math.cos(theta), 0.0],
                       [0.0,                0.0,           1.0]])
        return R 

    """
            Publish velocity message 
    """
    def setRobotSpeed(self):
        self.__vel_pub.publish(self.__vel_cmd)

    """
            Returns the publication rate 
    """
    def getRate(self):
        rate = rospy.Rate(self.__freq)
        return rate 
    
    """
            Returns the position of the robot based on odometry 
    """
    def getPosition(self):
        return np.array([[self.__x], [self.__y], [self.__z]])

    def getRotationMatrix(self):
        return self.__rot_mat
    
    def getTheta(self):
        return self.__theta

    def setLinearSpeed(self, l_speed):
        self.__vel_cmd.linear.x = l_speed 
    
    def setAngularSpeed(self, a_speed):
        self.__vel_cmd.angular.z = a_speed 
