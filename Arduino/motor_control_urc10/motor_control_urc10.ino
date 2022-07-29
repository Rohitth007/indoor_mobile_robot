#include "config.h"

//For serial communication
char receivedChars[12];
boolean newData = false;

//For time measurment
unsigned long currTime = 0, prevTime = 0; // time measurment
double dt = 0.0;                          //deltatime in seconds

//keeping it safe
long encoder_minimum = -2147483640;
long encoder_maximum = 2147483640;

//'Motor' Class for specific data corresponding to each motor
class Motor
{

public:
  float desiredSpeed = 0; // in revolutions per second
  float currentSpeed = 0; // in revolutions per second
  long encoderCount = 0;
  long prevEncoderCount = 0;
  int output;
  float gain = 0;

  void setDesiredSpeed(float s)
  // s is desired speed in revolutions per second
  {
    if (abs(s) > MAX_SPD)
    {
      int signofspeed = (s > 0) - (s < 0);
      desiredSpeed = signofspeed*MAX_SPD;
      return;
    }

    if (abs(s) < MIN_SPD)
    {
      desiredSpeed = 0;
      return;
    }
    desiredSpeed = s;
  }

  float computeOutput(float dt)
  //dt is deltatime in seconds
  {
    currentSpeed = ((encoderCount - prevEncoderCount) / (PPR * dt * GR)); //in revolutions per second
    prevEncoderCount = encoderCount;
    output = gain * (desiredSpeed - currentSpeed);
  }

  Motor(float motorGain)
  //Constructor for Motor Class
  {
    gain = motorGain;
  }
};

Motor leftMotor(left_motor_gain);
Motor rightMotor(right_motor_gain);

void setup()
{

  leftMotor.setDesiredSpeed(0.0);
  rightMotor.setDesiredSpeed(0.0);

  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  pinMode(RH_DIR, OUTPUT);
  pinMode(LH_DIR, OUTPUT);
  pinMode(RH_PWM, OUTPUT);
  pinMode(LH_PWM, OUTPUT);

  /* initialize hardware interrupts for RISE event
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderRiseEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderRiseEvent, RISING);
  */

  // initialize hardware interrupts for FALL event
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderFallEvent, FALLING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderFallEvent, FALLING);

  Serial.begin(9600);
  //Data expected in format<left_speed, right _speed>

  Serial.println("<Arduino is ready>");
}

void loop()
{
  //Get commanded speed
  recvWithStartEndMarkers();
  if (newData == true)
  {
    parseData();
    newData = false;
  }

  //Compute delta time
  currTime = micros();
  dt = (currTime - prevTime) / (1000000.0); // in seconds
  prevTime = currTime;

  //Run Proportional speed controller and get output
  rightMotor.computeOutput(dt);
  leftMotor.computeOutput(dt);

  //Apply control signal on the motors
  if (rightMotor.output > 0)
  {
    digitalWrite(RH_DIR, 1);
  }
  else
  {
    digitalWrite(RH_DIR, 0);
  }
  if (leftMotor.output > 0)
  {
    digitalWrite(LH_DIR, 1);
  }
  else
  {
    digitalWrite(LH_DIR, 0);
  }

  if (abs(rightMotor.output) > 255)
  {
    rightMotor.output = 255;
  }
  if (abs(rightMotor.output) < MIN_PWM)
  {
    rightMotor.output = 0;
  }
  if (abs(leftMotor.output) > 255)
  {
    leftMotor.output = 255;
  }
  if (abs(leftMotor.output) < MIN_PWM)
  {
    leftMotor.output = 0;
  }
  analogWrite(LH_PWM, abs(leftMotor.output));
  analogWrite(RH_PWM, abs(rightMotor.output));

  //return current tick count
  Serial.print("<");
  Serial.print(round(leftMotor.encoderCount));
  Serial.print(",");
  Serial.print(round(rightMotor.encoderCount));
  Serial.print(">\n");
}

/*============
void leftEncoderRiseEvent() 
{
  if (digitalRead(LH_ENCODER_B) == HIGH) 
  {
    if (leftMotor.encoderCount < encoder_maximum) 
      leftMotor.encoderCount++; 
  }
  else 
  {
    if (leftMotor.encoderCount > encoder_minimum) 
      leftMotor.encoderCount--;
  }
}
*/
//============
void leftEncoderFallEvent()
{
  if (digitalRead(LH_ENCODER_B) == LOW)
  {
    if (leftMotor.encoderCount < encoder_maximum)
      leftMotor.encoderCount++;
  }
  else
  {
    if (leftMotor.encoderCount > encoder_minimum)
      leftMotor.encoderCount--;
  }
}

/*============
void rightEncoderRiseEvent() 
{
  Serial.print("Right rise");
  if (digitalRead(RH_ENCODER_B) == HIGH)
  {
    Serial.println("B high");
    if (rightMotor.encoderCount < encoder_maximum) 
      rightMotor.encoderCount++;  
  }
  else 
  {
    Serial.println("B low");
    if (rightMotor.encoderCount > encoder_minimum) 
      rightMotor.encoderCount--;
  }
}
*/
//============
void rightEncoderFallEvent()
{
  if (digitalRead(RH_ENCODER_B) == LOW)
  {
    if (rightMotor.encoderCount < encoder_maximum)
      rightMotor.encoderCount--;
  }
  else
  {
    if (rightMotor.encoderCount > encoder_minimum)
      rightMotor.encoderCount++;
  }
}

//============
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

//============

void parseData()
// split the data into its parts
{
  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ","); // get the first part - left speed
  leftMotor.setDesiredSpeed(atoi(strtokIndx));

  strtokIndx = strtok(NULL, ","); // get the second part - right speed
  rightMotor.setDesiredSpeed(atoi(strtokIndx));
}
