#!/usr/bin/env python3

import rospy
import numpy as np
import math
from jackal_controll import *

if __name__ == "__main__":
    try:
        n = np.array([[1.0], [1.0], [0.0]])         # Trajectory vector
        f = 10                                      # Frequency of publishing 
        step = 1 / f                                # Period
        samples = 30 / step                         # Samples 
        t = np.linspace(0, 60, int(samples))        # Creation of time vector

        robot = Jackal(f)

        unit = np.eye(3)

        t = iter(t)

        rate = robot.getRate()

        while not rospy.is_shutdown():
            time = next(t, None)

            Pr = robot.getPosition()
            Pd = np.array([[4.0], [4.0], Pr[2]])

            error_p = Pd - Pr
            print("----") 
            print(error_p.T)
            R = robot.getRotationMatrix()
            theta = robot.getTheta()
            print("Theta = ", theta)


            error_b = R.T @ error_p
            print(error_b.T)
            unit_error = error_b / np.linalg.norm(error_b)


            dot = unit[0, :3] @ unit_error
            cross = np.cross(unit[0, :3], unit_error.T).reshape(3,1)
            n = cross[2] / np.linalg.norm(cross[2])

            e_a = n * math.acos(dot)
            print(e_a)


            robot.setAngularSpeed(4.5 * e_a)
            robot.setLinearSpeed(1 * error_b[0])

            robot.setRobotSpeed()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
