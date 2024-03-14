#include <Pixy2.h>
#include <math.h>

#include "MotorControls.h"
//#include <Odometry.h>

//Set pixy as main object
Pixy2 pixy;

// constants
const int conR = 10; // pin 10 on timer 1
const int conL = 9; // pin 9 on timer 1
const int conI = 3; // pins 11 and 3 on timer 2 (pin 11 seems to not work though)
const int frequency = 150; // do not change

// pixy variables
int x_pos_ooi = 158; //x position of object of interest
int y_pos_ooi = 316; //y position of object of interest

// Geometric Constants
const double closestY = 0.3; // [m] Closest physical distance the ball is from the robot and still in camera
const double farthestY = 3; // [m] Farthest distance ball is in camera view
const double widestX = 1.5; // [m] When ball is at farthest y, the widest x can be from center

// Positional variables
double x_pos = 0;
double y_pos = 0;

// control variables
double error = 0; // error of offset angle 
double prev_error = 0; // previous out for use with derivative controller (with compass this will get better)
double a_out = 0;
double integral = 0;
unsigned long last_time = 0;
int baseSpeed = 0;
int RightMotorSpeed = 0;
int LeftMotorSpeed = 0;

// Algorithm constants
// Speed
const int maxSpeed = 60; // (0 - 255) Use this to holistically adjust speed of robot, everything is based on this
//
const double Kp = 1*(0.5*maxSpeed); // Gain of PID system
const double Ki = 0.1*maxSpeed; // integral multiplier
const double Kd = 0.1*maxSpeed; // derivative multiplier
const double dt = 10; // time between error updates
const int noBallDelay = 200; // How long robot continues in current direction after loss of ball

void setup() {
  // initialize output pins

  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  pinMode(conI, OUTPUT);

  // start serial communication
  Serial.begin(9600);
  Serial.print("Starting...\n");

  //Initialize PWM
  InitTimersSafe();
  set_pwm_frequency(frequency);
  
  // intialize pixy library
  pixy.init();
  //StartOdometry();
}

void loop() {
    
    // Run intake constantly
    IMotor(conI, -40);

    /// GET PIXY
    int i;
    pixy.ccc.getBlocks();
    if (!pixy.ccc.numBlocks)
    {
      NoBalls(); // No Balls detected: continue in previous direction for some time before stopping
    } else{
      x_pos_ooi = pixy.ccc.blocks[0].m_x; // will focus on first seen object
      y_pos_ooi = pixy.ccc.blocks[0].m_y;
    }

    /// ON_BOARD BALL TRACKING CONTROL MAPPING

    // Map ball pos to real position on ground
    y_pos = map(y_pos_ooi, 0, 316, farthestY, closestY);
    x_pos = map(x_pos_ooi, 0, 316, -y_pos*widestX/farthestY, y_pos*widestX/farthestY);

    // set robot to move forward towards the ball
    baseSpeed = y_pos/farthestY*maxSpeed;
    
    // Find Angle of offset using x and y; angle of ball with respect to robot (we want this to be 0)
    error = atan(x_pos/y_pos); // converted error from sensor input into same units as desird value, i.e. radians/angle

    unsigned long now = millis();
    if (millis() > last_time + dt) {
      a_out = PID();
      last_time = millis();
    }
 
    // Set motor speeds
    RightMotorSpeed = baseSpeed - a_out;
    LeftMotorSpeed = baseSpeed + a_out;

    // Limit motors
    LimitMotors(maxSpeed*1.5);

    // Run Motors
    RMotor(conR, RightMotorSpeed); ////////NOTE adjusted right speed due to inequal resistance
    LMotor(conL, LeftMotorSpeed);

    /// PRINT Data
//    Serial.print("xpos: ");
//    Serial.print(x_pos_ooi);
//    Serial.print("ypos: ");
//    Serial.print(y_pos_ooi);
    Serial.print("Angle ofst: ");
    Serial.print(error);
    Serial.print("  R: ");
    Serial.print(RightMotorSpeed);
    Serial.print("   L: ");
    Serial.println(LeftMotorSpeed);
}

//END OF MAIN
//
//
//
///FUNCTIONS BELOW

// Changes PWM frequency for a given timer
void set_pwm_frequency(int frequency) {

  Timer1_Initialize();
  Timer2_Initialize();
  bool set_timer1_success = Timer1_SetFrequency(frequency);
  bool set_timer2_success = Timer2_SetFrequency(frequency);
  Serial.print("  Setting timer 1 frequency: ");
  Serial.print(set_timer1_success);
  Serial.print("  Setting timer 2 frequency: ");
  Serial.print(set_timer2_success);
}

// PID controller
double PID() {
  double proportional = error;
  integral += error*dt;
  double derivative = (error - prev_error) / dt;
  prev_error = error;
  double a_out = Kp*proportional + Ki*integral + Kd*derivative;
  return a_out;
}

// Scenario: No balls in sight
void NoBalls() {
  unsigned long noBallInitTime = millis();
  while(!pixy.ccc.numBlocks && millis() < noBallInitTime + noBallDelay) {
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
  }
  // Stop after timeout
  // Hold motors until ball found again
  while(!pixy.ccc.numBlocks){
    pixy.ccc.getBlocks();
    RMotor(conR, 0);
    LMotor(conL, 0);
    
    // PRINT Data
    Serial.print("Angle ofst: ");
    Serial.print(error);
    Serial.print("  R: ");
    Serial.print(RightMotorSpeed);
    Serial.print("   L: ");
    Serial.println(LeftMotorSpeed);
  }
}

// Limits the maximum speed of the motors to a chosen cap
void LimitMotors(int maxSpeed) {
  
    //If the motors are > maxSpeed, or < maxSpeed, we set speed to the max

    // Right
    if(RightMotorSpeed > maxSpeed)
    {
      RightMotorSpeed = maxSpeed;
    }
   
    if(RightMotorSpeed > maxSpeed)
    {
      RightMotorSpeed = maxSpeed;
    }

    if(RightMotorSpeed < -maxSpeed)
    {
      RightMotorSpeed = -maxSpeed;
    }
   
    if(RightMotorSpeed < -maxSpeed)
    {
      RightMotorSpeed = -maxSpeed;
    }

    // Left
    if(LeftMotorSpeed > maxSpeed)
    {
      LeftMotorSpeed = maxSpeed;
    }
   
    if(LeftMotorSpeed > maxSpeed)
    {
      LeftMotorSpeed = maxSpeed;
    }

    if(LeftMotorSpeed < -maxSpeed)
    {
      LeftMotorSpeed = -maxSpeed;
    }
   
    if(LeftMotorSpeed < -maxSpeed)
    {
      LeftMotorSpeed = -maxSpeed;
    }
}
