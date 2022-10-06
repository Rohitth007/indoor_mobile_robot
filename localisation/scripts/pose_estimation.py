

from xml.dom.expatbuilder import theDOMImplementation
import rospy
from std_msgs.msg import Int64
from math import sin, cos, pi, atan2, sqrt
import tf
from localisation.msg import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from numpy import *
from localisation.srv import odom_reset,odom_resetResponse

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
pose=(0,0,0)
def get_jacobian_state(pose,control,robo_width):
        theta = pose[2]
        l, r = control

        if r != l:
            alpha = (r - l) / robo_width
            R = l / alpha
            g1 = (R + (robo_width/ 2)) * (cos(theta + alpha) - cos(theta))
            g2 = (R + (robo_width/ 2)) * (sin(theta + alpha) - sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        else:

            m = array([[1.0, 0.0, -l * sin(theta)], [0.0, 1.0, l * cos(theta)], [0.0, 0.0, 1.0]])

        return m
def get_jacobian_control(pose,control,robo_width):
        theta = pose[2]
        l, r = tuple(control)

        if r != l:
            alpha = (r - l) / (robo_width)

            wr = ((robo_width) * r) / ((r - l) ** 2)
            wl = ((robo_width) * l) / ((r - l) ** 2)
            r2l = (r + l) / (2 * (r - l))

            g1_l = wr * (sin(theta + alpha) - sin(theta)) - r2l * cos(theta + alpha)
            g2_l = wr * (-cos(theta + alpha) + cos(theta)) - r2l * sin(theta + alpha)
            g3_l = - ((1 / robo_width))

            g1_r = -wl * (sin(theta + alpha) - sin(theta)) + r2l * cos(theta + alpha)
            g2_r = -wl * (-cos(theta + alpha) + cos(theta)) + r2l * sin(theta + alpha)
            g3_r = 1 / (robo_width)

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        else:

            # This is for the special case l == r.
            g1_l = .5 * (cos(theta) + (l / robo_width) * sin(theta))
            g2_l = .5 * (sin(theta) - (l / robo_width) * cos(theta))
            g3_l = - 1 / (robo_width)

            g1_r = .5 * (((-l / robo_width)) * sin(theta) + cos(theta))
            g2_r = .5 * (((l /robo_width)) * cos(theta) + sin(theta))
            g3_r = 1 / (robo_width)

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        return m
def get_covariance(covariance,current_pose,control,control_motion_factor,control_turn_factor,robot_width):
    
    left, right = control

    alpha_1 = control_motion_factor
    alpha_2 = control_turn_factor

    sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2
    sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2
    control_covariance = diag([sigma_l, sigma_r])
    G_t = get_jacobian_state(current_pose, control, robot_width)
    V   =  get_jacobian_control(current_pose, control,robot_width)

    covariance = dot(G_t, dot(covariance, G_t.T)) + dot(V, dot(control_covariance, V.T))
    return covariance

def pose_update(tick_difference,ticks_to_meter,width_robo,scanner_displacement):
	global pose
	#first case robot travels in straight line 
	if tick_difference[0]==tick_difference[1]:
		theta=pose[2]
		x=pose[0]+tick_difference[0]*ticks_to_meter*cos(theta)
		y=pose[1]+tick_difference[1]*ticks_to_meter*sin(theta)
		
		#returning the pose
		return (x,y,theta)


	#second case in case of a curve
	else:

		#getting the previous parameters
		theta=pose[2]
		x=pose[0]-scanner_displacement*sin(theta)
		y=pose[1]-scanner_displacement*cos(theta)
		

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

def set_odometry(req):
    global pose
    pose=(req.x,req.y,req.theta)

    return odom_resetResponse(True)

def main():
    global Tick
    global Posyx
    global pose
    #creating a publisher topic named pose and a node named pose_update 
    odom_pub = rospy.Publisher("odom",odometry_custom, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    lidar_broadcaster = tf.TransformBroadcaster()
    posyx_transform = tf.TransformBroadcaster()

    rospy.init_node('odom_node')


    s = rospy.Service('reset_odometry', odom_reset, set_odometry)


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

   
    covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])

    current_time = rospy.Time.now()
   
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)
        rospy.Subscriber("/pozyx_position",Twist,callback_posyx)

        tick_difference=[0,0]

        tick_difference[0]=Tick.left_tick-Tick.prev_left_tick
        tick_difference[1]=Tick.right_tick-Tick.prev_right_tick

        Tick.prev_left_tick=Tick.left_tick
        Tick.prev_right_tick=Tick.right_tick
        
        control = array((tick_difference[0],tick_difference[1])) * ticks_to_meter

        covariance = get_covariance(covariance,pose,control,control_motion_factor,control_turn_factor,width_robo)
        pose =pose_update(tick_difference,ticks_to_meter,width_robo,scanner_displacement)
        print("Pose")
        print("   ")
        print(pose)
        print("Covariance")
        print("   ")
        print(covariance)

        X=pose[0]
        Y=pose[1]
        theta=pose[2]


        X_posyx=Posyx.x
        Y_posyx=Posyx.y

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        


        odom_broadcaster.sendTransform((X, Y, 0),odom_quat,current_time,"/base_link","/odom")  
        lidar_broadcaster.sendTransform((X, Y, 0),odom_quat,current_time,"/laser","/odom") 
        posyx_transform.sendTransform((X_posyx,Y_posyx,0), odom_quat,current_time,"/posyx","/odom")

        odom = odometry_custom()
        odom.x = X
        odom.y = Y
        odom.theta = theta
        odom.covariance=covariance.flatten()
        
        # publish the message
        odom_pub.publish(odom)

        
        rate.sleep()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
