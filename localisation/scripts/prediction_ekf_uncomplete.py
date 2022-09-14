

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from math import sin, cos, pi, atan2, sqrt
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from numpy import *

#class to keep track of the ticks which arise from the motor encoders
class ExtendedKalmanFilter():
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor):

        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

    @staticmethod
    def dg_dstate(state, control, w):

        theta = state[2]
        l, r = control

        if r != l:
            alpha = (r - l) / w
            R = l / alpha
            g1 = (R + (w / 2)) * (cos(theta + alpha) - cos(theta))
            g2 = (R + (w / 2)) * (sin(theta + alpha) - sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        else:
            # This is for the special case r == l.
            m = array([[1.0, 0.0, -l * sin(theta)], [0.0, 1.0, l * cos(theta)], [0.0, 0.0, 1.0]])

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):

        theta = state[2]
        l, r = tuple(control)

        if r != l:
            alpha = (r - l) / w

            wr = (w * r) / ((r - l) ** 2)
            wl = (w * l) / ((r - l) ** 2)
            r2l = (r + l) / (2 * (r - l))

            g1_l = wr * (sin(theta + alpha) - sin(theta)) - r2l * cos(theta + alpha)
            g2_l = wr * (-cos(theta + alpha) + cos(theta)) - r2l * sin(theta + alpha)
            g3_l = - (1 / w)

            g1_r = -wl * (sin(theta + alpha) - sin(theta)) + r2l * cos(theta + alpha)
            g2_r = -wl * (-cos(theta + alpha) + cos(theta)) + r2l * sin(theta + alpha)
            g3_r = 1 / w

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        else:

            # This is for the special case l == r.
            g1_l = .5 * (cos(theta) + (l / w) * sin(theta))
            g2_l = .5 * (sin(theta) - (l / w) * cos(theta))
            g3_l = - 1 / w

            g1_r = .5 * ((-l / w) * sin(theta) + cos(theta))
            g2_r = .5 * ((l / w) * cos(theta) + sin(theta))
            g3_r = 1 / w

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        return m

    @staticmethod
    def get_error_ellipse(covariance):

        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):

        #subscribe with left and right tick and then cacualte tick difference like earlier
        left, right = control

        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor

        sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2
        sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2
        control_covariance = diag([sigma_l, sigma_r])
        
        G_t = self.dg_dstate(self.state, control, self.robot_width)
        V = self.dg_dcontrol(self.state, control, self.robot_width)

        self.covariance = dot(G_t, dot(self.covariance, G_t.T)) + dot(V, dot(control_covariance, V.T))

        # --->>> Put your code to compute the new self.state here.
        
        self.state = pose_update(self.state,control,self.robot_width,self.scanner_displacement)



class ticks:
    prev_left_tick=0
    prev_right_tick=0
    left_tick=0
    right_tick=0


Tick=ticks()

def pose_update(prev_pose,tick_difference,width_robo,scanner_displacement):

	#first case robot travels in straight line 
	if tick_difference[0]==tick_difference[1]:
		theta=prev_pose[2]
		x=prev_pose[0]+tick_difference[0]*cos(theta)
		y=prev_pose[1]+tick_difference[1]*sin(theta)
		
		#returning the pose
		return (x,y,theta)


	#second case in case of an arc
	else:

		#getting the previous parameters
		theta=prev_pose[2]
		x=prev_pose[0]-scanner_displacement*sin(theta)
		y=prev_pose[1]-scanner_displacement*cos(theta)
		

		alpha=(tick_difference[1]-tick_difference[0])/width_robo
		R=tick_difference[0]/alpha
		

        #calulating the center about which the curving happens 
		centerx=x-(R+width_robo/2)*sin(theta)
		centery=y+(R+width_robo/2)*cos(theta)
		theta=(theta + alpha + pi) % (2*pi) - pi
		
		#updating the x and using newly calualted theta value 
		x=centerx+(R+width_robo/2)*sin(theta)+scanner_displacement*sin(theta)
		y=centery-(R+width_robo/2)*cos(theta)+scanner_displacement*cos(theta)
		
		#print(x,y,theta)
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
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    lidar_broadcaster = tf.TransformBroadcaster()

    rospy.init_node('odom_node')


    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) # 1hz
  
    #hardware parameters 

    ticks_to_meter = rospy.get_param("ticks_to_meter") #in mm per tick
    width_robo = rospy.get_param("robot_width")  #in mm
    scanner_displacement = rospy.get_param("scanner_displacement")   #in mm
    control_motion_factor =  rospy.get_param("control_motion_factor")
    control_turn_factor = rospy.get_param("control_turn_factor")
    measurement_distance_stddev = rospy.get_param("measurement_distance_stddev")
    measurement_angle_stddev = rospy.get_param("measurement_angle_stddev")

    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    initial_state = array([0, 0, 0 ])

    ekf=ExtendedKalmanFilter(initial_state, initial_covariance,
                              width_robo, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)
    #begining pose estimation
    print("Starting Pose estimation")

    pose= (0, 0 ,0)
    states=[]
    covariances=[]
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)

        tick_difference=[0,0]

        tick_difference[0]=Tick.left_tick-Tick.prev_left_tick
        tick_difference[1]=Tick.right_tick-Tick.prev_right_tick

        Tick.prev_left_tick=Tick.left_tick
        Tick.prev_right_tick=Tick.right_tick
        
        control = array((tick_difference[0],tick_difference[1])) * ticks_to_meter

        

        '''pose =pose_update(pose,control,width_robo,scanner_displacement)
        print(pose)'''
        
        ekf.predict(control)

        pose=ekf.state
        covariance=ekf.covariance

        states.append(pose)
        covariances.append(ekf.covariance)

        #print(ekf.get_error_ellipse(covariance))

        x=pose[0]
        y=pose[1]
        theta=pose[2]

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        

        tx=x/10**3
        ty=y/10**3
        odom_broadcaster.sendTransform((tx, ty, 0),odom_quat,current_time,"/base_link","/odom")  
        lidar_broadcaster.sendTransform((tx, ty, 0),odom_quat,current_time,"/laser","/odom")  

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0,0))
        
        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        rate.sleep()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
