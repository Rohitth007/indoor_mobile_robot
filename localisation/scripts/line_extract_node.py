import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point 
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt 
import warnings
from localisation.msg import features
warnings.simplefilter('ignore')

def dist(p1,p2):
    d=((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    return d

def radius_negative_conversion(r,alpha):

    if r < 0:
        alpha = alpha + np.pi
        if alpha > np.pi:
            alpha = alpha-2*np.pi
        r = -r
    return r,alpha

#cartesian to polar conversion 
def cartesian_to_polar(X,Y,flag=False):

    if flag:
        plt.plot(X, Y, 'o', color='black')
    k,n = np.polyfit(X,Y,1)
    alpha = np.arctan(-1/k) 
    ro = n/(np.sin(alpha)-k*np.cos(alpha))
    return ro,alpha
#polar to cartesian conversion 
def polar_to_cartesian(r, theta):

    points_xy = np.array([np.cos(theta)*r, np.sin(theta)*r])

    return points_xy.T

def max_distant(points_xy):
    max_distance = 0
    max_index=-1
    for i in range(1,points_xy.shape[0]):

        p1=points_xy[0,:]
        pi=points_xy[i,:]
        p2=points_xy[-1,:]
        distance  = np.linalg.norm(np.cross(p2-p1, p1-pi))/np.linalg.norm(p2-p1)

        if(distance>max_distance):
            max_distance=distance
            max_index=i

    return max_index,max_distance

def split(points_xy, thereshold):
    max_index,max_distance = max_distant(points_xy)
   
   
    if max_distance>thereshold:
        points_xy_left = split(points_xy[:max_index+1,:],thereshold)
        points_xy_right = split(points_xy[max_index+1:,:],thereshold)

        points=np.vstack((points_xy_left,points_xy_right))
        #print(points.shape)
       
    else:
      
        points=np.vstack((points_xy[0,:],points_xy[-1,:]))
        #print(points.shape)

    return points

def make_marker(points):
    marker = Marker()
    marker.header.frame_id = "laser"
    marker.type = marker.LINE_LIST
    marker.action = marker.MODIFY

	# marker scale
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01

	# marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
	# marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

	# marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

	# marker line points
    marker.points = []
	# add points to show in RVIZ
    for i in range(points.shape[0]):
        p = Point()
        p.x,p.y,p.z = points[i,0],points[i,1],0
        marker.points.append(p)
    return marker

def merge(points):
    i=0

    #cleaning the data points based on a threshold
    while i<points.shape[0]-1:
        
        p1= points[i:i+2,0]
        p2= points[i:i+2,1]
        
        d=(p1[1]-p1[0])**2 + (p2[1]-p2[0])**2
        if(d<0.1):
            points[i]=0
            points[i+1]=0
        i+=2
        
    points = points[~np.all(points == 0, axis=1)]
    

    i=0
   # print(points.shape[0])
    params=[]

    #calculating parameters based on the cleaned set of line segements
    while i<points.shape[0]-1:
        #plt.plot(points[i:i+2,0],points[i:i+2,1])
        ro,alpha = cartesian_to_polar(points[i:i+2,0],points[i:i+2,1])
        params.append([ro,alpha])
        i+=2
    
    corellation_dict={}
    y=[]
    i=0
    
    while i<len(params):
        a1=params[i]
        j=i+1
        while j<len(params):
            a2=params[j]
            d=((a1[0]-a2[0])**2+(a1[1]-a2[1])**2)**0.5
            #print(d)
            if d<1.5:
                if i in corellation_dict.keys():
                        corellation_dict[i].append(j)
                else:
                        corellation_dict[i]=[j]
            j+=1
        i+=1
    
    for key in corellation_dict.keys():
        i1=2*key
        for i2 in corellation_dict[key]:
            ep11=points[i1]
            ep12=points[i1+1]
            ep21=points[2*i2]
            ep22=points[2*i2+1]
            d1=dist(ep12,ep21)
            d2=dist(ep11,ep22)
            #print("first",d1)
            #print("second",d2)

            if d1<0.15:
                points[i1+1]=ep22
                points[2*i2]=0
                points[2*i2+1]=0
        #print(" ")

    points = points[~np.all(points == 0, axis=1)]
    #print(corellation_dict)
    i=0
    params_clean=[]
    end_points=[]
    while i<points.shape[0]-1:
        
        plt.plot(points[i:i+2,0],points[i:i+2,1])
        ro,alpha = cartesian_to_polar(points[i:i+2,0],points[i:i+2,1],True)
        ro,alpha = radius_negative_conversion(ro,alpha)
        params_clean.append([ro,alpha])
        tp1=(points[i,0],points[i,1])
        tp2=(points[i+1,0],points[i+1,1])
        end_points.append([tp1[0],tp1[1]])
        end_points.append([tp2[0],tp2[1]])
        i+=2

    return params_clean,points,end_points

def split_and_merge(ranges,angles):

    split_threshold=0.012
    points_cartesian=polar_to_cartesian(np.array(ranges),np.array(angles))

    #spliiting the points and getting a collection of endpoints for line segments
    points=split(points_cartesian,split_threshold)
    #print(points)
    params_clean,points,end_points=merge(points)
    return params_clean,points,end_points

def callback(data):
    global pub
    #print(1)
    #getting the angle and ranges from the subscriber data
    ranges = np.asarray(data.ranges)
    angle_increment=data.angle_increment
    angle_max=data.angle_max+angle_increment
    angle_min=data.angle_min
    angles= np.array([np.arange(angle_min,angle_max,angle_increment)])[0]
    angles=angles[[ranges < 1E308]]
    ranges=ranges[[ranges < 1E308]]
    #print(angles)
    params_clean,points,end_points_list = split_and_merge(ranges, angles)
    
    radius=[]
    alpha=[]
    endpoints_x=[]
    endpoints_y=[]

    for e in params_clean:
        radius.append(e[0])
        alpha.append(e[1])
    
    for e in end_points_list:
        endpoints_x.append(e[0])
        endpoints_y.append(e[1])


    marker=make_marker(points)
    data_to_send = features()  # the data to be sent, initialise the array
    num_lines=(points.shape[0])/2
 
    #print(num_lines)
    #print(params_clean)
    #print(end_points_list)

    
    data_to_send.num_lines = int(points.shape[0]/2) # assign the array with the value you want to send
    
    data_to_send.radius_values=radius
    
    data_to_send.endpoints_x=endpoints_x

    data_to_send.endpoints_y=endpoints_y

    data_to_send.alpha_values=alpha

    #pub.publish(data_to_send)
    #rospy.loginfo("Hi")

    pub.publish(marker)


if __name__== "__main__":
   
    rospy.init_node('feature_from_lidar')
    pub = rospy.Publisher('visualization_msgs/MarkerArray',Marker,queue_size=10)
    #pub = rospy.Publisher('line_features', features, queue_size=10)
    print("starting scan analysis")
    rate=rospy.Rate(10)
    rospy.Subscriber("scan", LaserScan, callback)
    rate.sleep
    rospy.spin()
