#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import yaml

#Control Class
class Control:
    Kp = 0
    Ki = 0
    Kd = 0

#Class to store pose
class Pose:
    x = 0
    y = 0
    theta = 0

#Object initialization
GPose = Pose()
RPose = Pose() 

vControl = Control()
wControl = Control()

#Function to update robot pose
def update_pose(msg):

    global Rpose

    Rpose.x = msg.linear.x
    Rpose.y = msg.linear.y
    Rpose.theta = msg.angular.z

def get_distance(x1,y1,x2,y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def main():

    global RPose,GPose,vControl,wControl
    
    #Takes the user input
    if len(sys.argv) == 4:
        GPose.x = float(sys.argv[1])
        GPose.y = float(sys.argv[2])
        GPose.theta = float(sys.argv[3])
    else:
        print ("%s [x y theta]"%sys.argv[0])
        sys.exit(1)

    params = dict()
    with open(r'threshold_params.yaml') as file:
            params = yaml.load(file, Loader=yaml.FullLoader)

    control_pub = rospy.Publisher("cmd/vel",Twist,queue_size=1)

    #Subscriber to listen to the topic "pose"
    rospy.Subscriber('pose',Twist,update_pose) 

    control_msg = Twist()
    rate = rospy.Rate(1)

    goal_x = GPose.x
    goal_y = GPose.y
    goal_theta = GPose.theta

    while not rospy.is_shutdown:
        robot_x = RPose.x
        robot_y = RPose.y
        robot_theta = RPose.theta
        if(get_distance(robot_x,robot_y,goal_x,goal_y) >= params["distance_threshold"]):
            control_msg.linear.x = vControl.Kp*get_distance(robot_x,robot_y,goal_x,goal_y)
        else:
            control_msg.linear.x = 0
        if(math.abs(robot_theta-goal_theta) >= params["angular_threshold"]):        
            control_msg.angular.z = wControl.Kp*(robot_theta-goal_theta)
        else:
            control_msg.angular.z = 0
        control_pub.publish(control_msg)
        rate.sleep


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass    

