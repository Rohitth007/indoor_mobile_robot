import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
import serial
import time
import math
#Open the serial connection to the Arduino - which causes the Arduino to reset
ser = serial.Serial("/dev/ttyUSB0", 9600)

#For communication the message to be sent to the Arduino starts with startmarker '<' and ends with endmarker '>'
#the message content comprises desired left motor speed as integer and desired right motor speed as integer
startMarker = 60 #Unicode code for <
endMarker = 62 #Unicode code for >

#To store information from ROS callback
left_speed = 0
right_speed = 0
max_speed = 4
min_speed = 0.5
#Robot physical parms
wheel_radius = 37.0 #mm
robot_width = 475.0 #mm

#============================
def recvFromArduino():
# Receiving a message from the Arduino involves
# waiting until the startMarker is detected
# saving all subsequent bytes until the end marker is detected
  global startMarker, endMarker, ser

  recieved_data = ""
  recieved_char = "!" # any value that is not an endmarker (>) or startMarker (<)
  
  # wait for the start character
  while  ord(recieved_char) != startMarker: 
    recieved_char = ser.read()
  
  # save data until the end marker is found
  while ord(recieved_char) != endMarker:
    if ord(recieved_char) != startMarker:
      recieved_data = recieved_data + recieved_char.decode('utf-8') 
    recieved_char = ser.read()
  
  return(recieved_data)

#============================
# Wait for a message from the Arduino to give it time to reset
def waitForArduino():

  # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
  # it also ensures that any bytes left over from a previous message are discarded
  
  global startMarker, endMarker, ser
  print("Waiting for Arduino to be ready")
  
  msg = ""
  while msg.find("Arduino is ready") == -1:
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      pass
    
    msg = recvFromArduino()
 
  print(msg)
  return

#===========================
# Function for gracefull shutdown
def turn_off():
  #Function for safe shutdown 
  
  print('hw_interface node turning off')
  ser.write("<0,0>".encode('utf-8'))
  time.sleep(3)
  ser.close()
  return

#===========================
# ROS callback
def convert_vel_cmd(msg):
  #Function to convert linear and angular velocity request to 
  #left and right wheel velocities

  global wheel_radius, robot_width, left_speed, right_speed, max_speed, min_speed

  cmd_linear_vel = msg.linear.x*1000.0 # in mm/sec
  cmd_angular_vel = msg.angular.z # in rad/sec

  r_speed = ((2.0*cmd_linear_vel) + (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec
  l_speed = ((2.0*cmd_linear_vel) - (cmd_angular_vel*robot_width))/(2*wheel_radius) #in rad/sec
  
  r_speed = r_speed/(2*3.14) #in revolutions per second
  l_speed = l_speed/(2*3.14) #in revolutions per second
   
  vel_rl_max = max(r_speed,l_speed)
  vel_rl_min = min(r_speed,l_speed)
  
  if cmd_linear_vel != 0 and cmd_angular_vel != 0:
    if vel_rl_max > max_speed:
      r_speed = r_speed - (vel_rl_max - max_speed)
      l_speed = l_speed - (vel_rl_max - max_speed)
    elif vel_rl_min < -max_speed:
      r_speed = r_speed - (vel_rl_min + max_speed)
      l_speed = l_speed - (vel_rl_min + max_speed)
    if abs(r_speed) < min_speed or abs(l_speed) < min_speed:
      if abs(r_speed - l_speed) < 2*min_speed:
        r_speed = math.copysign(min_speed,r_speed)
        l_speed = math.copysign(min_speed,l_speed)
      else:
        if abs(r_speed) < min_speed:
          speed_sign = math.copysign(min_speed,r_speed)
          l_speed = l_speed + (speed_sign - r_speed)
          r_speed = speed_sign 
        if abs(l_speed) < min_speed:
          speed_sign = math.copysign(min_speed,l_speed)
          r_speed = r_speed + (speed_sign - l_speed)
          l_speed = speed_sign 	
  right_speed = r_speed
  left_speed = l_speed 	
  return

#============================
def main():
  global wheel_radius, robot_width, ser, left_speed, right_speed

  #Initialising             
  rospy.init_node('hw_interface')
  rospy.on_shutdown(turn_off)
  waitForArduino()
  
  print('hw_interface node running')
  rospy.Subscriber("/cmd_vel", Twist, convert_vel_cmd)
  pub_left_tick = rospy.Publisher("/left_ticks", Int64, queue_size = 1)
  pub_right_tick = rospy.Publisher("/right_ticks", Int64, queue_size = 1)
  
  wheel_radius = rospy.get_param('wheel_radius', 37)
  robot_width = rospy.get_param('robot_width', 475)

  rate = rospy.Rate(4) #2Hz
  while not rospy.is_shutdown():
    
    #Sent commanded velocity to Arduino
    output_string = "<" + str(int(left_speed*100)) + "," + str(int(right_speed*100)) + ">"
    print(output_string)
    ser.reset_output_buffer()
    ser.write(output_string.encode('utf-8'))
   
    #Update ticks info from arduino
    ser.reset_input_buffer()
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      pass

    recieved_data = recvFromArduino()
    print("a::",recieved_data)
    ticks = [int(x) for x in recieved_data.split(',')]
    
    print("ticks",ticks)
    #Publish ticks
    pub_left_tick.publish(ticks[0])
    pub_right_tick.publish(ticks[1])

    rate.sleep()

  return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
