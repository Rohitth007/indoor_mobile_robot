#!/usr/bin/env python
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 

#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x=[]
robot_position_y=[]
robot_orientation =[]

#defining two axes and figures for plotting 
fig , ax1 = plt.subplots()
fig2, ax2 = plt.subplots()

#finding full file path 
file_path = os.path.join(cwd_path, 'map_final.png')
img = plt.imread(file_path) #reading the background image 
ax1.imshow(img, extent=[0, 24000, 0, 16000]) #plotting it 
ax1.set(xlabel='X(m)', ylabel='Y(m)') #setting X and Y labels 

#plot robot
robot_1 = ax1.scatter(0.0, 0.0, alpha=1.0, s=50, color='green') # plotting the robot as a point 
X_cor = [0.0, 1000.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-') # plotting the intial heading from start to another point 


def pose_callback(data):

    #acessing the required global variables 
    global robot_position_x
    global robot_position_y
    global ax1

    #finding th erequired points to be plotted 
    y_start = data.linear.x + 200
    x_start = data.linear.y + 9280

    #filtering out unnesscary high values 
    if x_start<1e5 and y_start<1e5:

        #appending robots x and y position to list for plotting 
        robot_position_x.append(x_start)
        robot_position_y.append(y_start)
        
        #setting offsets to the oint being plotted as robot 
        robot_1.set_offsets([x_start, y_start ])  

        #converting orientation to radians 
        robot_orientation_radians = (data.angular.z-90)*(np.pi/180)

        #endpoint for the current heading 
        x_end = 1000*np.cos(robot_orientation_radians)+x_start
        y_end = 1000*np.sin(robot_orientation_radians)+y_start

        #updating the heading 
        heading_1.set_xdata([x_start, x_end])
        heading_1.set_ydata([y_start, y_end])

        #plotting the trajectory and collectin ghe orientation as a list 
        robot_orientation.append(robot_orientation_radians)
        ax2.plot(robot_orientation,color='y')
        ax1.plot(robot_position_x, robot_position_y, color='r', lw=2)

if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)

    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/pozyx_position", Twist, pose_callback)
    
    plt.show(block=True)
    rospy.spin()