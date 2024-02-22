#include <Pixy2.h>
#include <math.h>

#include "MotorControls.h"
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
const int conI = 3; // pins 11 and 3 on timer 2 (pin 11 seems to not work though)
const int frequency = 150; // do not change

// control variables
int RightMotorSpeed;
int LeftMotorSpeed;

// Algorithm constants
// Speed
const int maxSpeed = 50; // (0 - 255) Use this to holistically adjust speed of robot, all other mappings are based off this
//
const int yMax = maxSpeed*1.2; // Highest number y gets mapped to (slightly > overall cap speed)
const int ymin = 0; // lowest number y gets mapped to (some benefit to having a forward bias)
const int xErrMax = yMax*0.8; // Maximum absolute x error
const int quadraticXOvershootFactor = 1.3; // The quadratic pushes above the xmax on the edges
const int cubicXOvershootFactor = 1.3; // Same for cubic
const int chooseXMap = 2; // x mapping choice: Pick 1,2,3 to choose between linear qudratic and cubic
const int chooseYMap = 1; // y mapping choice: Pick 1 for linear, 2 for square root
const int noBallDelay = 750; // How long robot continues in current direction after loss of ball

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

    /// GET PIXY
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
    } else { // No Balls detected: continue in previous direction for some time before stopping
      NoBalls();
    }
    
    /// ON_BOARD BALL TRACKING CONTROL MAPPING

    // Map ball pos to motor cntrl vars
    x_error = map(x_position_ooi, 0, 316, -xErrMax, xErrMax);
    y_error = map(y_position_ooi, 0, 316, yMax, ymin);
    MapX();
    MapY();

    // Set motor speeds
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;

    // Limit motors to maxSpeed
    LimitMotors(maxSpeed);

    // Run intake constantly
    IMotor(conI, -40);


    /// PRINT Data
    Serial.print("X: ");
    Serial.print(x_error);
    Serial.print("  Y: ");
    Serial.print(y_error);
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

// x mapping function
void MapX() {
  if(chooseXMap == 2) {  // Quadratic
    if(x_error >= 0) {
      x_error = quadraticXOvershootFactor*pow(x_error, 2) / (xErrMax^2);
    } else {
      x_error = -quadraticXOvershootFactor*pow(abs(x_error), 2) / (xErrMax^2);;
    }
  } else if (chooseXMap == 3) { // Cubic
    x_error = cubicXOvershootFactor*pow(x_error, 3) / (xErrMax^3);
  } else {   // Otherwise Linear
    x_error = x_error;
  }
}

// y mapping function
void MapY() {
  if(chooseYMap == 2) { // Square root
    y_error = pow(y_error, 0.5)*pow(yMax, 0.5);
  } else { // Linear
    y_error = y_error;
  }
}

// Scenario: No balls in sight
void NoBalls() {
  unsigned long noBallInitTime = millis();
  while(!pixy.ccc.numBlocks && millis() < noBallInitTime + noBallDelay) {
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
  }
  RightMotorSpeed = 0;
  LeftMotorSpeed = 0;
  RMotor(conR, RightMotorSpeed);
  LMotor(conL, LeftMotorSpeed);
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
