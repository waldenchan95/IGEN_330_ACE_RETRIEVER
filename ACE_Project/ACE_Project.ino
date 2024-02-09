#include <Pixy2.h>
#include <math.h>

#include "MotorControls.h"
//#include "PixyControl.h"
//#include <Odometry.h>

//Set pixy as main object
Pixy2 pixy;

 int x_position_ooi; //x position of object of interest
 int y_position_ooi; //y position of object of interest

 int x_error = 0;
 int y_error = 0;

// drive variables

const int conR = 10;
const int conL = 9;
const int conI = 3;
const int frequency = 150;
// control variables

int RightMotorSpeed;
int LeftMotorSpeed;

void set_pwm_frequency(int frequency) {

  Timer2_Initialize();
  bool set_con_L_in = SetPinFrequencySafe(conL, frequency);
  bool set_con_I_in = Timer2_SetFrequency(frequency);
  Serial.print("  Setting conL Frequency: ");
  Serial.print(set_con_L_in);
  Serial.print("  Setting conI Frequency: ");
  Serial.print(set_con_I_in);
}

void setup() {
  // initialize output pins

  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  pinMode(conI, OUTPUT);

  // start serial communication
  Serial.begin(9600);
  Serial.print("Starting...\n");
  InitTimersSafe();
  set_pwm_frequency(frequency);

  delay(100);
  
  // intialize pixy library
  pixy.init();
  //StartOdometry();
}


void loop() {

      int i;
  // grab blocks!

  //pixy.ccc.getBlocks();   this was original
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
 
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

    x_error = int(pow(x_error, 3)/15000);
    y_error = y_error / 4;
    Serial.print("X_error: ");
    Serial.print(x_error);
    Serial.print("Y_error: ");
    Serial.print(y_error);
    
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;

    //If the motors are > 255, or < 255, we just set 255 to our maximum
    if(RightMotorSpeed > 150)
    {
      RightMotorSpeed = 150;
    }
   
    if(RightMotorSpeed > 150)
    {
      RightMotorSpeed = 150;
    }

    if(RightMotorSpeed < -150)
    {
      RightMotorSpeed = -150;
    }
   
    if(RightMotorSpeed < -150)
    {
      RightMotorSpeed = -150;
    }


    if(LeftMotorSpeed > 150)
    {
      LeftMotorSpeed = 150;
    }
   
    if(LeftMotorSpeed > 150)
    {
      LeftMotorSpeed = 150;
    }

    if(LeftMotorSpeed < -150)
    {
      LeftMotorSpeed = -150;
    }
   
    if(LeftMotorSpeed < -150)
    {
      LeftMotorSpeed = -150;
    }
    
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    IMotor(conI, -30);
    delay(10);

    //Odometry();
  }
