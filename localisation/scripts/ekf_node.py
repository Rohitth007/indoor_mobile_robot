from pyexpat import features
from statistics import covariance
from unittest import result
import rospy
from localisation.msg import *
from std_msgs.msg import Int64
#from math import sin, cos, pi, atan2, sqrt
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np


#class to keep track of the states and covariances
class ExtendedKalmanFilter():
    state=[]
    covariance=[]
    measurement_distance_stddev = 0.1
    measurement_angle_stddev = 0.1 
    threshold=0
    scanner_displacement=0
    result=[]
    map=dict()
    

    @staticmethod
    #fucntion which relates between the state and th reference walls 
    def h(state, landmark, scanner_displacement):
        #finding the r, theta of the wall withrespect to the lidar
        dx = landmark[0] - (state[0] + scanner_displacement * np.cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * np.sin(state[2]))
        r = np.sqrt(dx * dx + dy * dy)
        alpha = (np.atan2(dy, dx) - state[2] + np.pi) % (2*np.pi) - np.pi

        return np.array([r, alpha])
    
    @staticmethod
    #function to return the covariance detected parameters of the wall 
    def get_covariance(p1,p2,m_radius,m_angle):

        
        pho_1 = p1[0]**2+p1[1]**2
        theta_1= np.atan2(p1[1],p1[0])

        
        pho_2 = p2[0]**2+p2[1]**2
        theta_2= np.atan2(p2[0],p2[1])


        #covariance matrix for each of the two points used for estimaing walls
        covariance_x =np.diag[m_radius,m_radius,m_angle,m_angle]

        c=-((pho_1-pho_2)/2)*np.sin((theta_1-theta_2)/2)
        Fpq = np.array[[0,0,1/2,1/2],[np.cos((theta_1-theta_2)/2),-np.cos((theta_1-theta_2)/2),c,-c]]

        #converting the covarince to the final covariance estimate
        covariance_ar=np.dot(Fpq, np.dot(covariance_x, Fpq.T))

        return covariance_ar


    @staticmethod
    #fucntion to calculate the jacobian of the measurement with respect to the state 
    def dh_dstate(state, landmark, scanner_displacement):

        x, y, theta = state
        x_m, y_m = landmark
        d = scanner_displacement

        x_l = x + d * np.cos(theta)
        y_l = y + d * np.sin(theta)

        delta_x = x_m - x_l
        delta_y = y_m - y_l

        q = (delta_x) ** 2 + (delta_y) ** 2

        dr_dx = -delta_x / np.sqrt(q)
        dr_dy = -delta_y / np.sqrt(q)
        dr_dtheta = (d / np.sqrt(q)) * (delta_x * np.sin(theta) - delta_y * np.cos(theta))

        dalpha_dx = delta_y / q
        dalpha_dy = -delta_x / q
        dalpha_dtheta = - (d / q) * (delta_x * np.cos(theta) + delta_y * np.sin(theta)) - 1

        return np.array([[dr_dx, dr_dy, dr_dtheta], [dalpha_dx, dalpha_dy, dalpha_dtheta]])

    def lidar_correction(self,r_values,alpha_values,endpoints_x,endpoints_y):
        #loading the map
        map=self.map
        r_ref_values,alpha_ref_values = map_convert_to_polar(map)
        wall_begin_points=map["wall_begin_points"]
        wall_end_points=map["wall_end_points"]

        #converting to the world coordinate frame
        r_values,alpha_values,endpoints_x,endpoints_y= to_world(r_values,alpha_values,endpoints_x,endpoints_y)

        #defining certain thresholds to be used for calculation
        best_dist = self.threshold
        best_r = None

        #looping acroos the detected landmarks and refernce land marks to detect the best corellations 
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

                #set of conditions to ensure that the detected walls lie within the refernce walls 
                condition1 = endpoint_x_begin_wall<=endpoint_x_begin and endpoint_x_begin_wall<endpoint_x_end 
                condition2 = endpoint_x_end_wall>endpoint_x_begin and endpoint_x_end_wall>=endpoint_x_end 
    
                condition3 = endpoint_y_begin_wall<=endpoint_y_begin and endpoint_y_begin_wall<endpoint_y_end 
                condition4 = endpoint_y_end_wall>endpoint_y_begin and endpoint_y_end_wall>=endpoint_y_end 

                #detecting the best landmarks and adding them to  alist an returning it 
                if condition1 and condition2 and condition3 and condition4:
                    if dist_2 < best_dist:
                        best_dist = dist_2
                        best_r,best_alpha = r_ref_values[j],alpha_ref_values[j]
                        endpoint_b = [endpoint_x_begin,endpoint_y_begin]
                        endpoint_e = [endpoint_x_end,endpoint_y_end]
       
            if best_r:
                self.result.append(r_values[i],alpha_values[j],endpoint_b,endpoint_e,best_r,best_alpha)
        
        #using the list created to perform correction on the predicted estimate of the robots pose
        for ele in result:
            landmark=(ele[4]*np.cos(ele[5]),ele[4]*np.sin(ele[5]))

            #covarinace update and kalman gain calculation
            H_t = self.dh_dstate(self.state, landmark,self.scanner_displacement)
            Q = self.get_covariance(ele[2],ele[3],self.measurement_distance_stddev,self.measurement_angle_stddev)
            K_t = np.dot(np.dot(self.covariance, H_t.T), np.linalg.inv(np.dot(H_t, np.dot(self.covariance, H_t.T)) + Q))

            #correction step and updation of the state in the ekf class
            innovation = np.array(ele[0],ele[1]) - self.h(self.state, landmark, self.scanner_displacement)
            innovation[1] = (innovation[1] + np.pi) % (2*np.pi) - np.pi

            #updating the state and covariance
            self.state += np.dot(K_t, innovation)  

            self.covariance = np.dot(np.eye(3) - np.dot(K_t, H_t), self.covariance)

    def imu_correction(self,yaw_imu,variance_imu):
        yaw_imu = yaw_imu*np.pi/180
        current_yaw = self.state[3]
        current_variance = self.covariance[3][3]

        corrected_yaw = current_yaw + (current_variance/(current_variance + variance_imu))*(yaw_imu-current_yaw)

        corrected_variance = current_variance - (current_variance/(current_variance + variance_imu))*(current_variance)

        self.state[3]=corrected_yaw
        self.covariance[3][3]= corrected_variance

#function for proper conversion from catesian to polar if the radius is negative
def radius_negative_conversion(r,alpha):

    if r < 0:
        alpha = alpha + np.pi
        if alpha > np.pi:
            alpha = alpha-2*np.pi
        r = -r
    return r,alpha

#function to convert the detected landmark positions to the world coordinate frame for comparison
def to_world(r_values,alpha_values,endpoints_x,endpoints_y):
    global ekf
    r=[]
    alpha=[]
    endpoints_x_world=[]
    enpoints_y_world=[]
    dx = np.cos(ekf.state[2])
    dy = np.sin(ekf.state[2])

    #converting the radius and alpha values to world coordinate frame
    for index in len(r_values):

        x,y = r_values[index]*np.cos(alpha_values[index]),r_values[index]*np.sin(alpha_values[index])
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 
        r_world=np.sqrt(x_world**2+y_world**2)
        alpha_world=np.atan2(y_world,x_world)

        r.append(r_world)
        alpha.append(alpha_world)
    
    #converting the endpoints to the world coordinate frame
    for index in len(endpoints_x):

        x,y = endpoints_x[index],endpoints_y[index]
        x_world,y_world = (x * dx - y * dy + ekf.state[0], x * dy + y * dx + ekf.state[1]) 
        endpoints_x_world.append(x_world)
        enpoints_y_world.append(y_world)
    
    return r,alpha,endpoints_x,endpoints_y
    

#cartesian to polar conversion 
def cartesian_to_polar(X,Y):
    k,n = np.polyfit(X,Y,1)
    alpha = np.arctan(-1/k) 
    r = n/(np.sin(alpha)-k*np.cos(alpha))
    r,alpha=radius_negative_conversion(r,alpha)
    return r,alpha

#function to convert the map values from cartesian to polar coordiantes 
def map_convert_to_polar(map):
    wall_begin_points=map["wall_begin_points"]
    wall_end_points=map["wall_end_points"]
    r_values=[]
    alpha_values=[]

    for p1,p2 in wall_begin_points,wall_end_points:

        x=np.array(p1[0],p2[0])
        y=np.array(p1[1],p2[1])

        r,alpha=cartesian_to_polar(x,y)
        r_values.append(r)
        alpha_values.append(alpha)

    return r_values,alpha_values

def pred_update_callback(msg):
    #function to perform the prediction update step by using the messge from odom node
    global ekf
    ekf.state =np.array[msg.x,msg.y,msg.theta]
    ekf.covariance=msg.covariance


def correction_from_lidar_callback(msg):
    #storing the different features from the lfeature extract from lidar node
    global ekf
    ekf.lidar_correction(map,msg.radius_values,msg.alpha_values,msg.endpoints_x,msg.endpoints_y)

def correction_from_imu_callback(msg):
    #storing the different features from the lfeature extract from lidar node
    global ekf
    variance_imu = 5*np.pi/180
    ekf.imu_correction(msg.angular.z,variance_imu)


#objects of the required clases
ekf=ExtendedKalmanFilter()
ekf.map["wall_begin_points"] =rospy.get_param("wall_begin_points")
ekf.map["wall_end_points"] = rospy.get_param("wall_end_points")  

def main():

    #acessing the global class variables
    global ekf

    #creating a publisher topic named pose_ekf and a node named ekf_node
    ekf_pub = rospy.Publisher("pose_ekf", PoseWithCovarianceStamped, queue_size=1)
    ekf_broadcaster = tf.TransformBroadcaster() #tf broadcasters for odometry
    map=[]

    rospy.init_node('ekf_node')

    #initialising the rate variable to an appropriate rate 
    rate = rospy.Rate(10) 
  
    #intialising some parameters realted to the ekf 
    ekf.measurement_distance_stddev = rospy.get_param("measurement_distance_stddev")
    ekf.measurement_angle_stddev = rospy.get_param("measurement_angle_stddev")
    ekf.threshold=1.1

    #getting the map begin and end points


    #function to convert the map in cartesian to polar coordiante

    states=[]
    covariances=[]
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    #subscribing the required topics
    rospy.Subscriber('/odom1', odometry_custom, pred_update_callback)
    rospy.Subscriber('feature_from_lidar', features, correction_from_lidar_callback)
    rospy.Subscriber('/pozyx_position',Twist,correction_from_imu_callback)

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        
        states.append(ekf.state)
        covariances.append(ekf.covariance)

        X=ekf.state[0]
        Y=ekf.state[1]
        theta=ekf.state[2]


        #getting the quaternion for tf tranform concerning odometry and pozyx
        ekf_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        #preparing the message for publishing and publishing the message
        ekf_message = PoseWithCovarianceStamped()
        ekf_message.PoseWithCovarianceStamped.pose.position.x = X
        ekf_message.PoseWithCovarianceStamped.pose.position.y = Y
        ekf_message.PoseWithCovarianceStamped.pose.position.z = 0
        ekf_message.PoseWithCovarianceStamped.pose.orientation.x = ekf_quat[0]
        ekf_message.PoseWithCovarianceStamped.pose.orientation.y = ekf_quat[1]
        ekf_message.PoseWithCovarianceStamped.pose.orientation.z = ekf_quat[2]
        ekf_message.PoseWithCovarianceStamped.pose.orientation.w = ekf_quat[3]

        ekf_broadcaster.sendTransform((X, Y, 0),ekf_quat,current_time,"/ekf_base_link","/odom") 
        
        # publish the message
        ekf_pub.publish(ekf_message)

        last_time = current_time
        rate.sleep()
        
if __name__ == '__main__':
   
    try:
        main()
    except rospy.ROSInterruptException:
        pass
