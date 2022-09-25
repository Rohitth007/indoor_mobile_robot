from pyexpat import features
from statistics import covariance
from unittest import result
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
    measurement_distance_stddev = 0.1
    measurement_angle_stddev = 0.1 
    threshold=0
    scanner_displacement=0
    result=[]

    @staticmethod
    def h(state, landmark, scanner_displacement):
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])
    
    @staticmethod
    def get_covariance(p1,p2,m_radius,m_angle):
        pho_1 = p1[0]**2+p1[1]**2
        theta_1= atan2(p1[1],p1[0])

        pho_2 = p2[0]**2+p2[1]**2
        theta_2= atan2(p2[0],p2[1])

        covariance_x =diag[m_radius,m_radius,m_angle,m_angle]

        c=-((pho_1-pho_2)/2)*sin((theta_1-theta_2)/2)
        Fpq = array[[0,0,1/2,1/2],[cos((theta_1-theta_2)/2),-cos((theta_1-theta_2)/2),c,-c]]
        covariance_ar=dot(Fpq, dot(covariance_x, Fpq.T))

        return covariance_ar


    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):

        x, y, theta = state
        x_m, y_m = landmark
        d = scanner_displacement

        x_l = x + d * cos(theta)
        y_l = y + d * sin(theta)

        delta_x = x_m - x_l
        delta_y = y_m - y_l

        q = (delta_x) ** 2 + (delta_y) ** 2

        dr_dx = -delta_x / sqrt(q)
        dr_dy = -delta_y / sqrt(q)
        dr_dtheta = (d / sqrt(q)) * (delta_x * sin(theta) - delta_y * cos(theta))

        dalpha_dx = delta_y / q
        dalpha_dy = -delta_x / q
        dalpha_dtheta = - (d / q) * (delta_x * cos(theta) + delta_y * sin(theta)) - 1

        return array([[dr_dx, dr_dy, dr_dtheta], [dalpha_dx, dalpha_dy, dalpha_dtheta]])

    def correction(self,map,r_ref_values,alpha_ref_values,r_values,alpha_values,endpoints_x,endpoints_y):
        wall_begin_points=map["wall_begin_points"]
        wall_end_points=map["wall_end_points"]
        r_values,alpha_values,endpoints_x,endpoints_y= to_world(r_values,alpha_values,endpoints_x,endpoints_y)
        best_dist = self.threshold
        best_r = None

        for i in range(len(r_values)):
            endpoint_x_begin = endpoints_x[2*i][0]
            endpoint_y_begin = wall_begin_points[2*i][1]

            endpoint_x_end = wall_end_points[2*i+1][0]
            endpoint_y_end = wall_end_points[2*i+1][1]

            for j in range(len(r_ref_values)):
                dx, dy = r_ref_values[j] - r_values[i], alpha_ref_values[j] - alpha_values[i]

                dist_2 = dx * dx + dy * dy

                endpoint_x_begin_wall = wall_begin_points[j][0]
                endpoint_y_begin_wall = wall_begin_points[j][1]

                endpoint_x_end_wall = wall_end_points[j][0]
                endpoint_y_end_wall = wall_end_points[j][1]

                condition1 = endpoint_x_begin_wall<=endpoint_x_begin and endpoint_x_begin_wall<endpoint_x_end 
                condition2 = endpoint_x_end_wall>endpoint_x_begin and endpoint_x_end_wall>=endpoint_x_end 
    
                condition3 = endpoint_y_begin_wall<=endpoint_y_begin and endpoint_y_begin_wall<endpoint_y_end 
                condition4 = endpoint_y_end_wall>endpoint_y_begin and endpoint_y_end_wall>=endpoint_y_end 


                if condition1 and condition2 and condition3 and condition4:
                    if dist_2 < best_dist:
                        best_dist = dist_2
                        best_r,best_alpha = r_ref_values[j],alpha_ref_values[j]
                        endpoint_b = [endpoint_x_begin,endpoint_y_begin]
                        endpoint_e = [endpoint_x_end,endpoint_y_end]
            # If found, add to both lists.
            if best_r:
                self.result.append(r_values[i],alpha_values[j],endpoint_b,endpoint_e,best_r,best_alpha)
        
        for ele in result:
            landmark=(ele[0]*cos(ele[1]),ele[0]*sin(ele[1]))
            H_t = self.dh_dstate(self.state, landmark,self.scanner_displacement)
            Q = self.get_covariance(ele[2],ele[3],self.measurement_distance_stddev,self.measurement_angle_stddev)
            K_t = dot(dot(self.covariance, H_t.T), linalg.inv(dot(H_t, dot(self.covariance, H_t.T)) + Q))

            innovation = array(ele[0],ele[1]) - self.h(self.state, landmark, self.scanner_displacement)
            innovation[1] = (innovation[1] + pi) % (2*pi) - pi

            self.state += dot(K_t, innovation)  

            self.covariance = dot(eye(3) - dot(K_t, H_t), self.covariance)
      
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
    r=[]
    alpha=[]
    endpoints_x_world=[]
    enpoints_y_world=[]
    dx = cos(ekf.state[2])
    dy = sin(ekf.state[2])

    for index in len(r_values):

        x,y = r_values[index]*cos(alpha_values[index]),r_values[index]*sin(alpha_values[index])
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 
        r_world=sqrt(x_world**2+y_world**2)
        alpha_world=atan2(y_world,x_world)

        r.append(r_world)
        alpha.append(alpha_world)
    for index in len(endpoints_x):

        x,y = endpoints_x[index],endpoints_y[index]
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 
        endpoints_x_world.append(x_world)
        enpoints_y_world.append(y_world)
    
    return r,alpha,endpoints_x,endpoints_y
    

    

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
    ekf.threshold=1.1

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


        ekf.correction(map,r_ref_values,alpha_ref_values,features.radius_values,features.alpha_values,features.endpoints_x ,features.endpoints_y)
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
