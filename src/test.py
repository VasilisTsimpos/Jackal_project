#!/usr/bin/env python3

import rospy
import numpy as np
import math
from jackal_controll import *

if __name__ == "__main__":
    try:
        robot = Jackal(10)

        rate = robot.getRate()

        log_linear = []

        Pb = robot.getPosition()
        Pd = np.array([[0.9], [0.0], Pb[2]]) # Desired position from robots frame

        R0b = robot.getRotationMatrix()
        print(R0b)

        Pd = Pb + R0b @ Pd # Desired position from inertial axis

        while not rospy.is_shutdown():
            Pb = robot.getPosition()
            R0b = robot.getRotationMatrix()

            error = Pd - Pb

            errorb = R0b.T @ error

            linear_v = 0.5 * errorb[0]
            linear_v = 0.1
            log_linear.append(linear_v)

            #robot.setAngularSpeed(-0.3)
            robot.setLinearSpeed(linear_v)
            robot.calcVelocity()


            robot.setRobotSpeed()
            print(robot.log)

            rate.sleep()
    except rospy.ROSInterruptException:
        print("--End--")