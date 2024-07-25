#!/usr/bin/env python3

import rospy
import numpy as np
from jackal_controll import Jackal
import matplotlib.pyplot as plt
from tools import *

if __name__ == "__main__":
    robot = Jackal(10)
    time = 0
    tf = 12
    position_log = []
    trajectory_log = []
    time_log = []
    v_log = []
    velocity_log = []

    rate = robot.getRate()
        
    #Robots orientation
    R0b = robot.getRotationMatrix()
        
    #Robots position from inertial frame
    P0b = robot.getPosition()
    print('First Position', P0b[:2])

    #Robots desired position from robots frame
    Pd = np.array([2.0, -3.0, 0.0])
    size = np.linalg.norm(Pd)
    #Robots desired position from inertial frame
    P0d = P0b + changePerspective(Pd, R0b)

    try:
        while not rospy.is_shutdown():
            pos = robot.getPosition()

            pd, v, a = getFifthOrder(time, tf, P0b, 0, 0, P0d, 0, 0)
            v = np.linalg.norm(v)
            a = np.linalg.norm(a)

            e = getError(pos, pd, robot.getRotationMatrix()) # From robots frame

            lin_vel = v
            ang_vel = 0

            if time != 0:
                ang_vel = 0
                if time < tf:
                    ang_vel = 0.6 * getAngularError(e)
                lin_vel = 1 * getLinearError(e)

            if time >= tf + 2:
                raise Exception

            robot.setLinearSpeed(v)
            robot.setAngularSpeed(ang_vel)
            robot.setRobotSpeed()

            velocity = robot.calcLinearVel()

            time += 0.1

            #Log Data
            position_log.append(pos[:2])
            trajectory_log.append(pd[:2])
            velocity_log.append(velocity)
            v_log.append(v)
            time_log.append(time)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

    except Exception:
        print('Last Position: ', pos[:2])
        plt.figure()
        plt.plot(np.array(time_log), np.array(position_log))
        plt.plot(np.array(time_log), np.array(trajectory_log), linestyle='dashed')
        plt.xlabel('Time')
        plt.ylabel('Distance')
        plt.title('Position Of Jackal')
        plt.legend(['x', 'y','trajectory x', 'trajectory y'])
        plt.grid()

        plt.figure()
        plt.plot(np.array(time_log), np.array(v_log))
        plt.plot(np.array(time_log), np.array(velocity_log))
        plt.xlabel('Time')
        plt.ylabel('Velocity')
        plt.title('Velocity Of Jackal')
        plt.legend(['Trajectory velocity', 'Calculated velocity'])
        plt.grid()
        plt.show()

        print('Error in desired position', np.linalg.norm(P0b[:2] - pos[:2]) - size)