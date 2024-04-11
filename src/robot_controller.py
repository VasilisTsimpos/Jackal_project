#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

x = 0
y = 0
z = 0
xv = np.array([0,0,0])

R = np.identity(3)

def quaternion_to_rotation_matrix(orientation):
    """
    Convert quaternion to rotation matrix.

    Parameters:
        q (list or numpy array): Input quaternion [w, x, y, z].

    Returns:
        numpy array: 3x3 rotation matrix.
    """

    w = orientation.w 
    x = orientation.x
    y = orientation.y
    z = orientation.z

    # Edited by DP
    q = [ x, y, z, w]

    rotation = Rotation.from_quat(q)
    rotation_matrix = rotation.as_matrix()
    return rotation_matrix

def getJackalPosition(data):
    global x
    global y
    global z
    global xv
    global R

    # Extract the position information from the Odometry message
    position = data.pose.pose.position
    x = position.x
    y = position.y
    z = position.z

    # Extract the orientation information from the Odometry message
    orientation = data.pose.pose.orientation
    R = quaternion_to_rotation_matrix(orientation)
    xv = R[:, 0]


def move_jackal():
    # Initialize the ROS node
    rospy.init_node('jackal_controller', anonymous=True)

    # Wait for the Jackal system to start (adjust sleep duration as needed)
    rospy.sleep(3)
    print("--READY--")

    # Create a publisher for the Jackal's velocity topic
    velocity_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    # Create a subscriber for the Gazebo model states topic
    position_sub = rospy.Subscriber('/odometry/filtered', Odometry, getJackalPosition)

    n = np.array([[1], [2], [z]])    # Trajectory vector
    f = 10                      # Frequency of publishing 
    step = 1 / f                # Period
    samples = 60 / step         # Samples 
    t = np.linspace(0, 60, int(samples))
    Po = np.array([[0.0], [0.0], [z]])   # Starting point of the trajectory

    # Create a Twist message to send velocity commands
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.5  # Adjust the linear velocity as needed

    # Publish the message
    rate = rospy.Rate(f)  # 10 Hz

    t = iter(t)

    while not rospy.is_shutdown():
        time = next(t, None)
        
        Pd = Po + time * np.array([[0.0], [0.0] , [z]])
        Px = np.array([[x], [y], [z]])
        e_in = Pd - Px
        e_b = R.T @ e_in
        dist = np.linalg.norm(e_in, ord=2)
         
        cmd_vel_msg.linear.x = 4 * e_b[0]

        xd = e_b[0:2, 0]
        # print(xd)
        theta = 0.0
        if np.linalg.norm(xd) > 0.00000001:
            # print("Im in")
            theta = math.acos(xd @ np.array([[1],[0]]/np.linalg.norm(xd)))

        cmd_vel_msg.angular.z = 0
        if dist > 0.05:
            cmd_vel_msg.angular.z =  theta


        # print(cmd_vel_msg)
        velocity_pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_jackal()
    except rospy.ROSInterruptException:
        pass
