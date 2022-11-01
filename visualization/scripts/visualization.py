#!/usr/bin/env python
from turtle import position
import matplotlib.pyplot as plt
import numpy as np
import rospy
import time
import os
from geometry_msgs.msg import Twist

#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__))

#Setup plot
plt.ion()
robot_position_x=[]
robot_position_y=[]
robot_orientation =[]

fig , ax1 = plt.subplots()
fig2, ax2 = plt.subplots()
file_path = os.path.join(cwd_path, 'map_final.png')
img = plt.imread(file_path)
ax1.imshow(img, extent=[0, 24000, 0, 16000])
ax1.set(xlabel='X(m)', ylabel='Y(m)')

#plot robot
robot_1 = ax1.scatter(0.0, 0.0, alpha=1.0, s=50, color='green')
X_cor = [0.0, 100.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-')


def pose_callback(data):
    global robot_position_x
    global robot_position_y
    global ax1

    y_start = data.linear.x + 200
    x_start = data.linear.y + 9280

    if x_start<1e5 and y_start<1e5:

        robot_position_x.append(x_start)
        robot_position_y.append(y_start)
        
        robot_1.set_offsets([x_start, y_start ])  

        robot_orientation_radians = (data.angular.z-90)*(np.pi/180)

        x_end = 1000*np.cos(robot_orientation_radians)+x_start
        y_end = 1000*np.sin(robot_orientation_radians)+y_start

        heading_1.set_xdata([x_start, x_end])
        heading_1.set_ydata([y_start, y_end])

        robot_orientation.append(robot_orientation_radians)
        ax2.plot(robot_orientation,color='y')
        ax1.plot(robot_position_x, robot_position_y, color='r', lw=2)

if __name__ == '__main__':
    rospy.init_node('rover_visualisation', anonymous=True)

    rospy.Subscriber("/pozyx_position", Twist, pose_callback)
    
    plt.show(block=True)
    rospy.spin()