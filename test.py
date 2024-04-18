#!/usr/bin/env python3

import rospy
import numpy as np
import math
from jackal_controll import Jackal
import matplotlib.pyplot as plt

if __name__ == "__main__":
    try:
        robot = Jackal(10)

        x_log = []  
        odom_log = []
        v_log = []
        v_cmd_log = []
        a_log = []
        t_log = []
        calc_log = []
        time = 0
        tf = 5 

        rate = robot.getRate()

        Pb = robot.getPosition()
        Pb0 = Pb
        Pd = np.array([[1.0], [0.0], Pb[2]]) # desired position from robots frame
        
        R0b = robot.getRotationMatrix()

        Pd0 = Pb + R0b @ Pd # Desired position from the inertial frame 

        print("Iniitial pos: ", Pb.T, ", Desired pos: ", Pd0.T)

        while not rospy.is_shutdown():
            Pb = robot.getPosition()
            odom_log.append(Pb.reshape(3,).tolist())

            x,v,a = robot.getFifthOrder(time, tf, Pb0, 0, 0, Pd0, 0, 0)
            v = np.linalg.norm(v)
            if time > tf:
                x = Pd0
                v = 0
                # a  = 0
            x_log.append(x.reshape(3,).tolist()) 
            v_log.append(v)
            Pd = x
            
            R0b = robot.getRotationMatrix()

            error = Pd-Pb # Error from the inertial frame 

            errorb = R0b.T @ error # Error from the robots frame

            linear_v = 10 * errorb[0]
            v_cmd_log.append(linear_v)
            
            calc_log.append(robot.calcVelocity())

            robot.setLinearSpeed(linear_v)

            
            robot.setRobotSpeed()        

            t_log.append(time)
            time += 0.1
            rate.sleep()
    except rospy.ROSInterruptException:
        t = np.array(t_log)
        x = np.array(x_log).T
        odom = np.array(odom_log).T
        vel = np.array(v_log)
        calc_vel = np.array(calc_log)
        cmd_vel = np.array(v_cmd_log)


        fig, ax = plt.subplots(2)
        ax[0].plot(t, x[0,:])
        ax[0].plot(t, x[1,:])
        ax[0].plot(t, odom[0,:])
        ax[0].plot(t, odom[1,:])
        ax[0].set(xlabel='Time', ylabel='Position')
        ax[0].legend(['X trajectory position',
                      'Y trajectory positino',
                      'X robot position',
                      'Y robot position'])
        ax[0].grid()


        ax[1].plot(t, vel)
        ax[1].plot(t, calc_vel)
        ax[1].plot(t, cmd_vel)
        ax[1].set(xlabel='Time', ylabel='Velocity')
        ax[1].legend(['Trajectory velocity',
                      'Calculated velocity',
                      'Commanded velocity'])
        ax[1].grid()
        plt.show()

        print("\n")
        print("--End--") 