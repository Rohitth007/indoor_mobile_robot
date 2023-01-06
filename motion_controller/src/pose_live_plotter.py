import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

anchor_pos = []
x_robot = 0
y_robot = 0
theta_robot = 0

def update_pose(msg):
    global x_robot,y_robot,theta_robot
    x_robot = msg.linear.x
    y_robot = msg.linear.y
    theta_robot = msg.angular.z
    
if __name__ == "__main__":
    rospy.init_node("position_plot_node")
    time = rospy.Rate(10)

    map_sub = rospy.Subscriber("\pozyx_position",Twist,update_pose)

    plt.xlabel('x-axis')
    plt.ylabel('y-axis')

    while not rospy.is_shutdown():
        plt.plot(x_robot,y_robot)
        time.sleep()
    plt.show()    
