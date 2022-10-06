#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from motion_controller.msg import odometry_custom
import math
import yaml

#Control Class
class Control:
    Kp = 1.0
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

    global RPose

    #RPose.x = msg.x
    #RPose.y = msg.y
    RPose.theta = msg.angular.z

def get_distance(x1,y1,x2,y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

  
def main():

    global RPose,GPose,vControl,wControl
    
    rospy.init_node('go_to_goal_node')
    
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
    rospy.Subscriber('/pozyx_position',odometry_custom,update_pose) 

    control_msg = Twist()
    rate = rospy.Rate(1)

    goal_x = GPose.x
    goal_y = GPose.y
    goal_theta = GPose.theta

    while not rospy.is_shutdown():

        robot_x = RPose.x
        robot_y = RPose.y
        robot_theta = RPose.theta
        
        if(math.abs(goal_theta-robot_theta) >= params["angular_threshold"]):
                control_msg.angular.z = wControl.Kp*(goal_theta-robot_theta)
            else:
                control_msg.angular.z = 0
                return 
                
	"""
        if(get_distance(robot_x,robot_y,goal_x,goal_y) >= params["distance_threshold"]):

            control_msg.linear.x = vControl.Kp*get_distance(robot_x,robot_y,goal_x,goal_y)

            theta_error  = math.atan2((goal_y-robot_y),(goal_x-robot_x)) - robot_theta
            theta_error = (theta_error+math.pi)%(2*math.pi) - math.pi

            
        
        else:
            
            control_msg.linear.x = 0
            
            if(math.abs(goal_theta-robot_theta) >= params["angular_threshold"]):
                control_msg.angular.z = wControl.Kp*(goal_theta-robot_theta)
            else:
                control_msg.angular.z = 0 
	"""
        control_pub.publish(control_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass    

