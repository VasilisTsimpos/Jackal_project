#!/usr/bin/env python3

import rospy
import numpy as np
import math
from jackal_controll import Jackal
import matplotlib.pyplot as plt

if __name__ == "__main__":
    try:
        robot = Jackal(10)

        log = []
        v_log = []
        time = 0
        unit = np.eye(3)

        rate = robot.getRate()

        Pb = robot.getPosition()
        Pd = np.array([[3.0], [-1.0], Pb[2]]) # desired position from robots frame
        
        R0b = robot.getRotationMatrix()

        Pd = Pb + R0b @ Pd # Desired position from the inertial frame 

        while not rospy.is_shutdown():
            Pb = robot.getPosition()
            
            R0b = robot.getRotationMatrix()

            error = Pd-Pb # Error from the inertial frame 

            errorb = R0b.T @ error # Error from the robots frame

            unit_error = errorb / np.linalg.norm(errorb)


            dot = unit[0, :3] @ unit_error
            cross = np.cross(unit[0, :3], unit_error.T).reshape(3,1)
            n = cross[2] / np.linalg.norm(cross[2])

            e_a = n * math.acos(dot)

            linear_v = 0.8 * errorb[0]
            v_log.append(linear_v)

            robot.setLinearSpeed(linear_v)
            robot.setAngularSpeed(e_a)
            # robot.setLinearSpeed(0.1)

            
            vb = robot.calcVelocity()
            log.append(vb)
            robot.setRobotSpeed()        

            time += 0.1
            rate.sleep()
    except rospy.ROSInterruptException:
        print("\n")
        t = np.linspace(0, time, len(log))
        y = np.array(log)
        k = np.array(v_log)
        plt.plot(t, y, color = 'blue')
        plt.plot(t, k, color = 'red')
        plt.legend(["calculated velocity", "commanded velocity"])
        plt.show()
        print("--End--") 