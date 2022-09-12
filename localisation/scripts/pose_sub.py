#!/usr/bin/env python

#importing the necessary libraries and other functions 
import rospy
from localisation.msg import features

#defining the call back function 
def callback(data):
    print(1)
    rospy.loginfo(rospy.get_caller_id() + 'X : %s ,Y : %s ,Theta %s', data.num_lines)

#creating fuction to guide the subscribing process 
def sub_pose():
    #creating a node named sub_pose for subscribing the pose estimates
    print(1)
    rospy.init_node('sub_pose',anonymous=True)
    rospy.Subscriber('line_features', features, callback)

    rospy.spin()

if __name__ == '__main__':
    sub_pose()
