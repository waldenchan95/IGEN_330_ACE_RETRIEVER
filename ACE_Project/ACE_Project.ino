#include <Pixy2.h>
#include <math.h>

#include <MotorControls.h>
#include <PixyControl.h>

//Set pixy as main object
Pixy2 pixy;

// initialize variables

int x_position_ooi; //x position of object of interest
int y_position_ooi; //y position of object of interest

// drive variables

const int conR = 10;
const int conL = 9;
const int frequency = 150;
// control variables

int x_error = 0;
int y_error = 0;

int RightMotorSpeed;
int LeftMotorSpeed;

void setup() {
  // initialize output pins

  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);

  // start serial communication
  Serial.begin(9600);
  Serial.print("Starting...\n");

  set_pwm_frequency(conL, frequency);

  delay(100);
  
  // intialize pixy library
  pixy.init();
}


void loop() {
  // put your main code here, to run repeatedly:

    int i;
  // grab blocks!
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

    x_error = int(pow(x_error, 3)/5000);
   
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;

    //If the motors are > 255, or < 255, we just set 255 to our maximum
    if(RightMotorSpeed > 255)
    {
      RightMotorSpeed = 255;
    }
   
    if(RightMotorSpeed > 255)
    {
      RightMotorSpeed = 255;
    }

   
    if(RightMotorSpeed < -255)
    {
      RightMotorSpeed = -255;
    }
   
    if(RightMotorSpeed < -255)
    {
      RightMotorSpeed = -255;
    }


    if(LeftMotorSpeed > 255)
    {
      LeftMotorSpeed = 255;
    }
   
    if(LeftMotorSpeed > 255)
    {
      LeftMotorSpeed = 255;
    }

   
    if(LeftMotorSpeed < -255)
    {
      LeftMotorSpeed = -255;
    }
   
    if(LeftMotorSpeed < -255)
    {
      LeftMotorSpeed = -255;
    }
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
  }
