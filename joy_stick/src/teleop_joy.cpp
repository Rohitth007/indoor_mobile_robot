#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string.h>

struct node_helper
{
  int axis_linear, axis_angular,axis_autonomous;
  double scale_linear, scale_angular;
  bool isAutonomous = false;
  geometry_msgs::Twist cmd_vel;

  void joy_Callback(const sensor_msgs::Joy::ConstPtr &joy)
  {
    cmd_vel.angular.z = scale_angular * joy->axes[axis_angular];
    cmd_vel.linear.x = scale_linear * joy->axes[axis_linear];
    if(joy->axes[axis_autonomous] < -0.5)
    	isAutonomous = true;
    else
    	isAutonomous = false;	
  }

  void diag_Callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &diag)
  { 
    auto msg = diag->status[0].message;
    if(msg == "Joystick not open."){
         cmd_vel.angular.z = 0;
         cmd_vel.linear.x = 0;
         isAutonomous = false;
    }
  }

  //constructor to initialise
  node_helper()
  {
    axis_linear = 1;
    axis_angular = 0;
    axis_autonomous = 5;
    scale_linear = 1.0;
    scale_angular = 0.35;
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.x = 0.0;
  }

} joy_helper;

//The main
int main(int argc, char **argv)
{

  //Initialise node
  ros::init(argc, argv, "teleop_joystick");
  ros::NodeHandle nh;

  //Create params
  nh.param("axis_linear", joy_helper.axis_linear, joy_helper.axis_linear);
  nh.param("axis_angular", joy_helper.axis_angular, joy_helper.axis_angular);
  nh.param("scale_angular", joy_helper.scale_angular, joy_helper.scale_angular);
  nh.param("scale_linear", joy_helper.scale_linear, joy_helper.scale_linear);

  //Create publisher
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  //Create subscriber for topic "joy"
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, &node_helper::joy_Callback, &joy_helper);

  //Create subscriber for topic "diagnostic"
  ros::Subscriber diag_sub = nh.subscribe("diagnostics", 10, &node_helper::diag_Callback, &joy_helper);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    if(!joy_helper.isAutonomous)
    	vel_pub.publish(joy_helper.cmd_vel);
    loop_rate.sleep();
  }
  std::cout << "shutting down teleop_joy node\n";
  return 0;
}


