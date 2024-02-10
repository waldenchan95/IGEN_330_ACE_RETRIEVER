#include <Pixy2.h>
#include <math.h>

#include "MotorControls.h"
//#include "PixyControl.h"
//#include <Odometry.h>

//Set pixy as main object
Pixy2 pixy;

// pixy variables
int x_position_ooi; //x position of object of interest
int y_position_ooi; //y position of object of interest

int x_error = 0;
int y_error = 0;

// constants
const int conR = 10; // pin 10 on timer 1
const int conL = 9; // pin 9 on timer 1
const int conI = 3; // pins 11 and 3 on timer 2
const int frequency = 150;
const int maxSpeed = 56;

// control variables
int RightMotorSpeed;
int LeftMotorSpeed;

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

    int i;
    pixy.ccc.getBlocks();
   
    if (pixy.ccc.numBlocks)
    {
      //Serial.print("Detected ");
      //Serial.print(pixy.ccc.numBlocks);
      //Serial.println(" objects");
      for (i=0; i<pixy.ccc.numBlocks; i++)
      {
          x_position_ooi = pixy.ccc.blocks[i].m_x;
          y_position_ooi = pixy.ccc.blocks[i].m_y;
      }
    }
 
    x_error = map(x_position_ooi, 0, 316, -100, 100);
    y_error = map(y_position_ooi, 0, 316, 400, -150);

    x_error = int(pow(x_error, 3)/22000);
    y_error = y_error / 10;
    Serial.print("X_error: ");
    Serial.print(x_error);
    Serial.print("Y_error: ");
    Serial.print(y_error);
    
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;

    // cap motor speed to maxSpeed
    LimitMotors(maxSpeed);
    
    RMotor(conR, RightMotorSpeed/1.4);
    LMotor(conL, LeftMotorSpeed/1.4z);
    IMotor(conI, -30);

    //Odometry();
}

// Changes PWM frequency for a gtiven timer
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
