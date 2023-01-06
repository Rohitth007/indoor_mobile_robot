//Pins for Motor 1
#define RH_ENCODER_A 3 
#define RH_ENCODER_B 9
#define RH_PWM 6
#define RH_DIR 7

//Pins for Motor 2
#define LH_ENCODER_A 2
#define LH_ENCODER_B 8
#define LH_PWM 5
#define LH_DIR 4

#define GR 50.9 // Gear Ratio
#define PPR 7.0 // Pulse Per Revolution

#define MIN_PWM 25
#define MAX_PWM 255
#define MIN_SPD 0.4 // in revolutions per second
#define MAX_SPD 4 // in revolutions per second

// Proportional speed controller
#define left_motor_P_gain 0.5
#define right_motor_P_gain 0.5 

// Integral speed controller
#define left_motor_I_gain 0.0 
#define right_motor_I_gain 0.0 

// Derivative speed controller
#define left_motor_D_gain 0.0
#define right_motor_D_gain 0.0
//Only the falling edge can be detected and we are only detecting 
// falling edge on one channel so that would be 2 pulse per revolution
