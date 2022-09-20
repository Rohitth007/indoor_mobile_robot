

from xml.dom.expatbuilder import theDOMImplementation
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from math import sin, cos, pi, atan2, sqrt
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from numpy import *

#class to keep track of the ticks which arise from the motor encoders
class ticks:
    prev_left_tick=0
    prev_right_tick=0
    left_tick=0
    right_tick=0

class posyx:
    x=0
    y=0 

Tick=ticks()
Posyx =posyx()

def get_control_covariance(control,control_motion_factor,control_turn_factor):
    
    left, right = control

    alpha_1 = control_motion_factor
    alpha_2 = control_turn_factor

    sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2
    sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2
    control_covariance = [sigma_l, sigma_r]

    return control_covariance

def pose_update(prev_pose,tick_difference,ticks_to_meter,width_robo,scanner_displacement):
	
	#first case robot travels in straight line 
	if tick_difference[0]==tick_difference[1]:
		theta=prev_pose[2]
		x=prev_pose[0]+tick_difference[0]*ticks_to_meter*cos(theta)
		y=prev_pose[1]+tick_difference[1]*ticks_to_meter*sin(theta)
		
		#returning the pose
		return (x,y,theta)


	#second case in case of a curve
	else:

		#getting the previous parameters
		theta=prev_pose[2]
		x=prev_pose[0]-scanner_displacement*sin(theta)
		y=prev_pose[1]-scanner_displacement*cos(theta)
		

		alpha=ticks_to_meter*(tick_difference[1]-tick_difference[0])/width_robo
		R=ticks_to_meter*tick_difference[0]/alpha
		

        #calulating the center about which the curving happens 
		centerx=x-(R+width_robo/2)*sin(theta)
		centery=y+(R+width_robo/2)*cos(theta)
		theta=(theta + alpha + pi) % (2*pi) - pi
		
		#updating the x and using newly calualted theta value 
		x=centerx+(R+width_robo/2)*sin(theta)+scanner_displacement*sin(theta)
		y=centery-(R+width_robo/2)*cos(theta)+scanner_displacement*cos(theta)
		
		
		return (x,y,theta)
    
def callback_left(msg):
    global Tick
    Tick.left_tick=msg.data

def callback_right(msg):
    global Tick
    Tick.right_tick=msg.data

def callback_posyx(msg):
    global Posyx
    Posyx.x=msg.linear.x
    Posyx.y=msg.linear.y
        
def main():
    global Tick
    global Posyx
    #creating a publisher topic named pose and a node named pose_update 
    odom_pub = rospy.Publisher("odom", Twist, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    lidar_broadcaster = tf.TransformBroadcaster()
    posyx_transform = tf.TransformBroadcaster()

    rospy.init_node('odom_node')


    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) # 1hz
  
    #hardware parameters 

    ticks_to_meter = rospy.get_param("ticks_to_meter") #in mm per tick
    width_robo = rospy.get_param("robot_width")  #in mm
    scanner_displacement = rospy.get_param("scanner_displacement")   #in mm
    control_motion_factor =  rospy.get_param("control_motion_factor")
    control_turn_factor = rospy.get_param("control_turn_factor")
   
    #begining pose estimation
    print("Starting Pose estimation")

    pose= (0, 0 ,0)

    current_time = rospy.Time.now()
   
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)
        rospy.Subscriber("/position",Twist,callback_posyx)

        tick_difference=[0,0]

        tick_difference[0]=Tick.left_tick-Tick.prev_left_tick
        tick_difference[1]=Tick.right_tick-Tick.prev_right_tick

        Tick.prev_left_tick=Tick.left_tick
        Tick.prev_right_tick=Tick.right_tick
        
        control = array((tick_difference[0],tick_difference[1])) * ticks_to_meter

    
        pose =pose_update(pose,tick_difference,ticks_to_meter,width_robo,scanner_displacement)
        
        print(pose)

        control_covariance = get_control_covariance(control,control_motion_factor,control_turn_factor)

        X=pose[0]
        Y=pose[1]
        theta=pose[2]


        X_posyx=Posyx.x
        Y_posyx=Posyx.y

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        


        odom_broadcaster.sendTransform((X/10**3, Y/10**3, 0),odom_quat,current_time,"/base_link","/odom")  
        lidar_broadcaster.sendTransform((X/10**3, Y/10**3, 0),odom_quat,current_time,"/laser","/odom") 
        posyx_transform.sendTransform((X_posyx/10**3,Y_posyx/10**3,0), odom_quat,current_time,"/posyx","/odom")

        odom = Twist()
        odom.linear.x = X/10**3
        odom.linear.y = Y/10**3
        odom.angular.z = theta
        
        # publish the message
        odom_pub.publish(odom)

        
        rate.sleep()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
