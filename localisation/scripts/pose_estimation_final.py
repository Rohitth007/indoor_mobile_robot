import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from math import sin, cos, pi
import yaml

class ticks:
    prev_left_tick=0
    prev_right_tick=0
    left_tick=0
    right_tick=0


Tick=ticks()

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
		theta+=alpha
		
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
        
def main():
    global Tick
    #creating a publisher topic named pose and a node named pose_update 
    pub = rospy.Publisher('pose', Twist, queue_size=1)
    rospy.init_node('pose_update')
    
    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(1) # 1hz
    msg = Twist()

    #getting the data and inital parameters
    
    
    #hardware parameters 

    ticks_to_meter = rospy.get_param("ticks_to_meter") #in mm per tick
    width_robo = rospy.get_param("robot_width")  #in mm
    scanner_displacement = rospy.get_param("scanner_displacement")   #in mm
    
    
    #begining pose estimation
    print("Starting Pose estimation")

    pose= (0, 0 ,0 )


    while not rospy.is_shutdown():
        
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)

        tick_difference=[0,0]

        tick_difference[0]=Tick.left_tick-Tick.prev_left_tick
        tick_difference[1]=Tick.right_tick-Tick.prev_right_tick

        pose =pose_update(pose,tick_difference,ticks_to_meter,width_robo,scanner_displacement)

        X=pose[0]
        Y=pose[1]
        theta=pose[2]
            
        msg.linear.x = X
        msg.linear.y = Y
        msg.angular.z = theta
        pose=(X,Y,theta)

        Tick.prev_left_tick=Tick.left_tick
        Tick.prev_right_tick=Tick.right_tick

        pub.publish(msg)
            
        rate.sleep
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
