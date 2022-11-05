import rospy
from geometry_msgs.msg import Twist
import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200)

#For communication the message to be sent to the Arduino starts with startmarker '<' and ends with endmarker '>'
#the message content comprises desired left motor speed as integer and desired right motor speed as integer
startMarker = 60 #Unicode code for <
endMarker = 62 #Unicode code for >

pos = Twist()

#Number pf values for taking average
mov_avg_length = 10
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

#Function for safe shutdown 
def turn_off():
  print('pozyx_interface node turning off')
  time.sleep(3)
  ser.close()
  return

#===========================    	
def main():
  global ser, pos
  
  x_avg = []
  y_avg = []
  theta_avg = [] 
	
  #Initialising             
  rospy.init_node('pozyx_interface_node')
  rospy.on_shutdown(turn_off)
  waitForArduino()
  
  print('pozyx_interface node running')
  pub_position = rospy.Publisher("/pozyx_position",Twist, queue_size = 1)
 
  rate = rospy.Rate(10) #10Hz
  while not rospy.is_shutdown():
    #Update ticks info from arduino
    ser.reset_input_buffer()
    #Wait until something comes into the serial recieve buffer
    while (ser.in_waiting == 0): 
      pass

    recieved_position_data = recvFromArduino()
    
    position = [int(x) for x in recieved_position_data.split(',')]
    
    
    if position[0] < 1e5 and position[1] < 1e5:

      x_avg.append(position[0])
      y_avg.append(position[1])
      theta_avg.append(position[2])

      #Computing the moving average of the pose
      if len(x_avg) == mov_avg_length:
        pos.linear.x = sum(x_avg)/len(x_avg)
        x_avg.pop(0)
      else:
        pos.linear.x = position[0]

      if len(y_avg) == mov_avg_length:
        pos.linear.y = sum(y_avg)/len(y_avg)
        y_avg.pop(0)
      else:
        pos.linear.y = position[1]

      if len(theta_avg) == mov_avg_length:
        pos.angular.z = sum(theta_avg)/len(theta_avg)
        theta_avg.pop(0)
      else:
        pos.angular.z = position[2]    

      pub_position.publish(pos)

    rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
 
