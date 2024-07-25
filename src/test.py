#!/usr/bin/env python3

import rospy
import numpy as np
from jackal_controll import Jackal
import matplotlib.pyplot as plt
from tools import *

if __name__ == "__main__":
    robot = Jackal(10)
    time = 0
    position_log = []
    time_log = []
    v_log = []
    velocity_log = []

    rate = robot.getRate()
        
    #Robots orientation
    R0b = robot.getRotationMatrix()
        
    #Robots position from inertial frame
    P0b = robot.getPosition()

    #Robots desired position from robots frame
    Pd = np.array([1.5, 1.0, 0.0])
    #Robots desired position from inertial frame
    P0d = P0b + changePerspective(Pd, R0b)

    try:
        while not rospy.is_shutdown():
            pos = robot.getPosition()

            e = getError(pos, P0d, robot.getRotationMatrix()) # From robots frame
            
            pd, v, a = getFifthOrder(time, 8, P0b, 0, 0, P0d, 0, 0)
            v = np.linalg.norm(v)
            a = np.linalg.norm(a)

            ang_vel = 0.7 * getAngularError(e)            
            if np.linalg.norm(e[:2]) < 0.05:
                ang_vel = 0
                raise Exception

            robot.setLinearSpeed(v)
            robot.setAngularSpeed(ang_vel)
            robot.setRobotSpeed()

            velocity = robot.calcLinearVel()

            time += 0.1

            #Log Data
            velocity_log.append(velocity)
            position_log.append(pos)
            v_log.append(v)
            time_log.append(time)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

    except Exception:
        print('Last Position: ', pos[:2])
        plt.figure()
        plt.plot(np.array(time_log), np.array(v_log))
        plt.plot(np.array(time_log), np.array(velocity_log))
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.legend(['Commanded velocity', 'Calculated velocity'])
        plt.grid()
        plt.show()
