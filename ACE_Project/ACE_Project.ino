#include <Pixy2.h>
#include <math.h>

#include "MotorControls.h"
//#include <Odometry.h>

//Set pixy as main object
Pixy2 pixy;

// pixy variables
int x_pos_ooi = 158; //x position of object of interest
int y_pos_ooi = 316; //y position of object of interest
int last_x_pos_ooi = 158; //previous position used by
int last_y_pos_ooi = 316;
int xVelCor = 0;
int yVelCor = 0;
int lastPosTime;

int x_error = 0;
int y_error = 0;

// constants
const int conR = 10; // pin 10 on timer 1
const int conL = 9; // pin 9 on timer 1
const int conI = 3; // pins 11 and 3 on timer 2 (pin 11 seems to not work though)
const int frequency = 150; // do not change

// control variables
int RightMotorSpeed = 0;
int LeftMotorSpeed = 0;

// Algorithm constants
// Speed
const int maxSpeed = 60; // (0 - 255) Use this to holistically adjust speed of robot, all other mappings are based off this
//
const int yMax = maxSpeed*1.2; // Highest number y gets mapped to (slightly > overall cap speed)
const int ymin = 0.3*maxSpeed; // lowest number y gets mapped to (some benefit to having a forward bias)
const int xErrMax = maxSpeed*0.6; // Maximum absolute x error [factor is essentially side to side K value]
const double quadraticXOvershootFactor = 1; // The quadratic pushes above the xmax on the edges
const double cubicXOvershootFactor = 1.3; // Same for cubic
const int chooseXMap = 1; // x mapping choice: Pick 1,2,3,4 to choose between linear qudratic and cubic, and 4 for custom arctan function
const int chooseYMap = 1; // y mapping choice: Pick 1 for linear, 2 for square root
const int noBallDelay = 0; // How long robot continues in current direction after loss of ball
const double YpctForXSlow = 0.2; // This determines the percentage of y error under which x error is reduced (reduces x sensitivity when ball very close)
const double pctXReduce = 0.5; // This determines the percent x is reduced to when the ball is very close
const double xVelCorFactor = 0.8; // How aggressively does the corrected position lead real current position (1 being exactly as much as previous)
const double yVelCorFactor = 0.8; // ^

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
      x_pos_ooi = pixy.ccc.blocks[0].m_x;
      y_pos_ooi = pixy.ccc.blocks[0].m_y;
    }

    /// VELOCITY CORRECTION

    DynPosCorrection();

    /// ON_BOARD BALL TRACKING CONTROL MAPPING

    // Map ball pos to motor cntrl vars
    x_error = map(x_pos_ooi + xVelCor, 0, 316, -xErrMax, xErrMax);
    y_error = map(y_pos_ooi + yVelCor, 0, 316, yMax, ymin);
    MapX();
    if(y_error < YpctForXSlow*yMax) {
      x_error = x_error*pctXReduce;
    }
    MapY();

    // Set motor speeds
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;

    // Limit motors to maxSpeed
    LimitMotors(maxSpeed);

    // Run Motors
    RMotor(conR, RightMotorSpeed*0.76); ////////NOTE adjusted right speed due to inequal resistance
    LMotor(conL, LeftMotorSpeed*1.05);

    /// PRINT Data
//    Serial.print("xpos: ");
//    Serial.print(x_pos_ooi);
//    Serial.print("ypos: ");
//    Serial.print(y_pos_ooi);
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
  } else if(chooseXMap == 3) { // Cubic
    x_error = cubicXOvershootFactor*pow(x_error, 3) / (xErrMax^3);
  } else if(chooseXMap == 4) { // arctan based; near linear with ramp up in the centre
    if(x_error > 1) {
      x_error = 2*(maxSpeed/5)*(atan(x_error/(1.3*(maxSpeed/5)) - 2) + 1.1);
    } else if(x_error < -1) {
      x_error = 2*(maxSpeed/5)*(atan(-x_error/(1.3*(maxSpeed/5)) - 2) + 1.1);
    } else {
      x_error = 0;
    }
  } else {   // Otherwise Linear
    if(abs(x_error) < 4){
      x_error = 0;
    }
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
  // Stop after timeout
  RightMotorSpeed = 0;
  LeftMotorSpeed = 0;
  x_error = 0;
  y_error = 0;
  // Hold motors until ball found again
  while(!pixy.ccc.numBlocks){
    pixy.ccc.getBlocks();
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    
    // PRINT Data
    Serial.print("X: ");
    Serial.print(x_error);
    Serial.print("  Y: ");
    Serial.print(y_error);
    Serial.print("  R: ");
    Serial.print(RightMotorSpeed);
    Serial.print("   L: ");
    Serial.println(LeftMotorSpeed);
  }
}

void DynPosCorrection() {
  if(x_pos_ooi > last_x_pos_ooi + 90 || y_pos_ooi > last_y_pos_ooi + 90) { // If the new position is vastly far from old position, something odd has happened like ball lost and found again so reset
    last_x_pos_ooi = x_pos_ooi;
    last_y_pos_ooi = y_pos_ooi;
  }

  if(millis() > lastPosTime + 130) {
    
    xVelCor = xVelCorFactor*(x_pos_ooi - last_x_pos_ooi);
    yVelCor = yVelCorFactor*(y_pos_ooi - last_y_pos_ooi);
    
    // Set new "last" variables
    last_x_pos_ooi = x_pos_ooi;
    last_y_pos_ooi = y_pos_ooi;
    lastPosTime = millis();
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
