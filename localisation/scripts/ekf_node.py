from pyexpat import features
from statistics import covariance
import rospy
from localisation.msg import *
from std_msgs.msg import Int64
from math import sin, cos, pi, atan2, sqrt
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from numpy import *

#class to keep track of the ticks which arise from the motor encoders
class ExtendedKalmanFilter():
    state=[]
    covariance=[]
    measurement_distance_stddev = 0
    measurement_angle_stddev = 0 

    def correction(self,map,r_ref_values,alpha_ref_values,r_values,alpha_values,endpoints_x,endpoints_y):
        wall_begin_points=map["wall_begin_points"]
        wall_end_points=map["wall_end_points"]
        r_values,alpha_values,endpoints_x,endpoints_y= to_world(r_values,alpha_values,endpoints_x,endpoints_y)
class Features:
    radius_values=[]
    alpha_values=[]
    endpoints_x=[]
    endpoints_y=[]
    number_lines=0

def radius_negative_conversion(r,alpha):

    if r < 0:
        alpha = alpha + pi
        if alpha > pi:
            alpha = alpha-2*pi
        r = -r
    return r,alpha

def to_world(r_values,alpha_values,endpoints_x,endpoints_y):
    global ekf
    pass
    

#cartesian to polar conversion 
def cartesian_to_polar(X,Y):


    k,n = polyfit(X,Y,1)
    alpha = arctan(-1/k) 
    r = n/(sin(alpha)-k*cos(alpha))
    r,alpha=radius_negative_conversion(r,alpha)
    return r,alpha

def map_convert_to_polar(map):
    wall_begin_points=map["wall_begin_points"]
    wall_end_points=map["wall_end_points"]
    r_values=[]
    alpha_values=[]

    for p1,p2 in wall_begin_points,wall_end_points:

        x=array(p1[0],p2[0])
        y=array(p1[1],p2[1])

        r,alpha=cartesian_to_polar(x,y)
        r_values.append(r)
        alpha_values,append(alpha)
    return r_values,alpha_values

def pred_update_callback(msg):
    global ekf
    ekf.state =array[msg.x,msg.y,msg.theta]
    ekf.covariance=msg.covariance


def feature_update_callback(msg):
    global features
    features.radius_values = msg.radius_values
    features.alpha_values = msg.alpha_values
    features.endpoints_x = msg.endpoints_x
    features.endpoints_y = msg.endpoints_y

ekf=ExtendedKalmanFilter()  
features= Features()

def main():
    global ekf
    global features
    #creating a publisher topic named pose and a node named pose_update 
    ekf_pub = rospy.Publisher("pose_ekf", odometry_custom, queue_size=1)
    map=[]

    rospy.init_node('ekf_node')


    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) 
  
    #hardware parameters 
    ekf.measurement_distance_stddev = rospy.get_param("measurement_distance_stddev")
    ekf.measurement_angle_stddev = rospy.get_param("measurement_angle_stddev")
    map=dict()
    map["wall_begin_points"] =rospy.get_param("wall_begin_points")
    map["wall_end_points"] = rospy.get_param("wall_end_points")

    r_ref_values,alpha_ref_values = map_convert_to_polar(map)


    #begining pose estimation
    print("Starting Pose estimation")

    pose= (0, 0 ,0)
    states=[]
    covariances=[]
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        rospy.Subscriber('/odom', odometry_custom, pred_update_callback)
        rospy.Subscriber('feature_from_lidar', features, feature_update_callback)


        ekf.correctio(map,r_ref_values,alpha_ref_values,features.radius_values,features.alpha_values,features.endpoints_x ,features.endpoints_y)
        states.append(ekf.state)
        covariances.append(ekf.covariance)


        msg = odometry_custom()
        msg.x = ekf.state[0]
        msg.y = ekf.state[1]
        msg.theta = ekf.state[2]
        msg.covariance=ekf.covariance
        
        # publish the message
        ekf_pub.publish(msg)


        last_time = current_time
        rate.sleep()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
