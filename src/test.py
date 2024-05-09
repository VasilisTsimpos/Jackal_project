#!/usr/bin/env python3

import rospy
import numpy as np
import math
from jackal_controll import Jackal
import matplotlib.pyplot as plt

def getLinearError(p, pd, R):
        error = Pd-Pb # Error from the inertial frame 

        errorb = R.T @ error # Error from the robots frame

        return errorb[0]

def make_unit(p):
    return p / np.linalg.norm(p)


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
        tf = 10
        kp = 4.16

        rate = robot.getRate()

        Pb0 = Pb = robot.getPosition()
        Pd = np.array([[1.0], [1.0], [0.0]]) # desired position from robots frame

        dot_product = np.eye(3)[0, :2] @ make_unit(Pd[:2, 0]).reshape(2,1)
        cross_product = np.cross(np.eye(3)[0, :3], make_unit(Pd).T).reshape(3,1)
        n = 1
        if (abs(cross_product[2]) > 0.01):
            n = cross_product[2] / np.linalg.norm(cross_product[2])
        Ad0 = n * math.acos(dot_product) # Desired angle

        Pd0 = Pb + robot.getRotationMatrix() @ Pd # Desired position from the inertial frame 
        print("Desired pos ", Pd0.T)

        while not rospy.is_shutdown():
            Pb = robot.getPosition()
            R0b = robot.getRotationMatrix()

            
            Pd,v,a = robot.getFifthOrder(time, tf, Pb0, 0, 0, Pd0, 0, 0)
            v = np.linalg.norm(v)
            a = np.linalg.norm(a)

            Ad,w,ca = robot.getFifthOrder(time, 1, 0, 0, 0, Ad0, 0, 0)

            linear_v = kp * getLinearError(Pb, Pd, R0b) # Proportional error
            
            robot.setLinearSpeed(linear_v) # Set linear speed
            robot.setAngularSpeed(w) # Set angular speed
            
            robot.setRobotSpeed() # Publish speed

            # Log data 
            calc_log.append(robot.calcVelocity())
            odom_log.append(Pb.reshape(3,).tolist())
            v_cmd_log.append(linear_v)
            x_log.append(Pd.reshape(3,).tolist()) 
            v_log.append(v)
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


        fig, ax = plt.subplots(2, figsize = (10,10))
        ax[0].plot(t, x[0,:], linestyle = 'dashed')
        ax[0].plot(t, x[1,:], linestyle = 'dashed')
        ax[0].plot(t, odom[0,:])
        ax[0].plot(t, odom[1,:])
        ax[0].set(xlabel='Time', ylabel='Position')
        ax[0].legend(['X trajectory position',
                      'Y trajectory positino',
                      'X robot position',
                      'Y robot position'])
        ax[0].grid()


        ax[1].plot(t, vel)
        ax[1].plot(t, cmd_vel)
        ax[1].set(xlabel='Time', ylabel='Velocity')
        ax[1].legend(['Trajectory velocity',
                      'Commanded velocity'])
        ax[1].grid()

        d = np.linalg.norm(Pb0[:2,0] - Pd0[:2,0])
        fig.suptitle(f'Distance traveled: {d:.2f}m, tf: {tf}sec, Kp: {kp}')
        plt.show()

        print("\n")
        print("--End--") 