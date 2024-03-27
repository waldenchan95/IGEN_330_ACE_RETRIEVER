// NEED TO ADD:
// e_stop and e_home state logic, startup angle set to 0 and communicated with external system; 
// Once odometry is tested: point target error, hold angle during WAIT state

#include <Pixy2.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "MotorControls.h"

//Set pixy as main object
Pixy2 pixy;

// OVERALL CONTROL
// States
enum States { WAIT, GOTO_PT, GOTO_BALL, GET_BALL, CHK_DONE, GO_HOME, HOLD } state;
// Commands
enum Commands { start, e_stop, e_home } command;
// Controlled externally: tells robot if there are any more balls it must go to
bool balls = 0;

// Variables
unsigned long init_ball_seen_time; // When robot sees ball in pixy it waits a short time to determine if the object is consistent before going

// Odometry Definitions
#define CLKS_PER_SAMPLE 4 // pure counts of encoder
#define DIST_PER_CLK 0.005984734 // 3inch wheel, 40 clicks per rotation (m)

// PINS
// Motor
const int conR = 6; // all should be on timer 5 of the mega
const int conL = 7; // 
const int conI = 44; // 
// Rotary Encoder Module connections
const int rDT = 5;    // DATA signal 
const int rCLK = 19;    // CLOCK signal
const int lDT = 4;    // DATA signal 
const int lCLK = 18;    // CLOCK signal

// Motor PWM Frequency
const int input_frequency = 150; // do not change

// ODOMETRY VARIABLES
// Store previous Pins state
int rPreviousCLK;   
int rPreviousDATA;
int lPreviousCLK;   
int lPreviousDATA;
// Store current counter value
int rcounter = 0;
int lcounter = 0;
// Position variables
double x = 0; //(m)
double y = 0; //(m)
double a = 0; //(rad)
// Magnetometer
// Hard-iron calibration settings
const float hard_iron[3] = {
  -12.04,  -15.64,  13.31
};
// Soft-iron calibration settings
const float soft_iron[3][3] = {
  {  1.0,  0.0, 0.0  },
  {  0.0,  1.0, 0.0  },
  {  0.0,  0.0, 1.0  }
};
static float heading = 0;

// TRACKING VARIABLES
// pixy variables
int x_pos_ooi = 158; //x position of object of interest
int y_pos_ooi = 316; //y position of object of interest
// Ball in Camera Positional variables
double x_ball_pos = 0;
double y_ball_pos = 0;
// control variables
double a_error = 0; // a_error of offset angle 
double prev_a_error = 0; // previous out for use with derivative controller (with compass this will get better)
double a_out = 0;
double a_integral = 0;
double d_error = 0;
double prev_d_error = 0;
double d_integral = 0;
double baseSpeed = 0; // this is like a_out for d
unsigned long last_time = 0;
int RightMotorSpeed = 0;
int LeftMotorSpeed = 0;
int IntakeSpeed = 0;

// CONSTANTS
// Geometric Constants
const long closestY = 20; // [cm] Closest physical distance the ball is from the robot and still in camera
const long farthestY = 162; // [cm] Farthest distance ball is in camera view
const long widestX = 126; // [cm] When ball is at farthest y, the widest x can be from center
// Algorithm constants
// Speed
const double maxSpeed = 100; // (0 - 255) Use this to holistically adjust speed of robot, everything is based on this
// PID
const double dt = 0.005; // (s) time between a_error updates (multiplied by 1000 to be used in millis()) 
// Angle PID constants
const double aKp = 1.4*maxSpeed; // Gain of P
const double aKi = 1.2*maxSpeed; // integral multiplier
const double aKd = 0.016*maxSpeed; // derivative multiplier
// BaseSpeed PID constants
const double dKp = 0.6*maxSpeed; // Gain of P
const double dKi = 0.34*maxSpeed; // integral multiplier
const double dKd = 0.02*maxSpeed; // derivative multiplier

//Create instance of magnetometer
Adafruit_LIS3MDL lis3mdl;

void setup() {
  // initialize output pins
  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  pinMode(conI, OUTPUT);
  // start serial communication
  Serial.begin(115200);
  Serial.print("Starting...\n");
  //Initialize PWM
  set_pwm_frequency(input_frequency);
  // intialize pixy library
  pixy.init();
  // Setup Odometry
  StartOdometry();
  // First state
  state = WAIT;
}

void loop() {
    /// EXECUTE ODOMETRY
    Odometry();

    /// STATEMACHINE
    switch(state) {
      case WAIT:
        // Wait for signal
        if (command == start) {
          state = GOTO_PT;
          init_ball_seen_time = millis();
        }
        else if (command == e_home)
          state = GO_HOME;
        else
          state = WAIT;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        RightMotorSpeed = 0;
        LeftMotorSpeed = 0;
        IntakeSpeed = 0;
        // WILL ADD: once odometry works will add fucntionality to maintain starting angle.
        // This will also revert the robot back to the correct angle after coming home during the GO_HOME state
      break;
      case GOTO_PT:
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks && millis() > init_ball_seen_time + 500) {
            state = GOTO_BALL;
        } else {
          init_ball_seen_time = millis(); // when ball is seen this will be the initial time it was seen
          state = GOTO_PT;
        }
        /// EXTERNAL CAMERA/POINT TARGET
        IntakeSpeed = 0; // DO NOT Run intake when ball NOT in view
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
              // Do things based on external camera
              // Set targets:
              // Overwrite camera-based ball positions and angle error with other points and angles for robot to go to
              // Compute angle between bot and target, target 0
              // Compute distance from point, target 0
        /// APPROACH TARGET
        // Check control feedback
        if (millis() > last_time + dt*1000) {
          a_out = PID(a_error, prev_a_error, a_integral, aKp, aKi, aKd, dt);
          baseSpeed = PID(d_error, prev_d_error, d_integral, dKp, dKi, dKd, dt);
          last_time = millis();
        }
        // Set motor speeds
        RightMotorSpeed = baseSpeed - a_out;
        LeftMotorSpeed = baseSpeed + a_out;
      break;
      case GOTO_BALL:
        IntakeSpeed = 40; // Run intake when ball in view
        
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks) {
          state = GOTO_BALL;
        } else {
          state = GET_BALL;
          init_ball_seen_time = millis();
        }
          
        x_pos_ooi = pixy.ccc.blocks[0].m_x; // will focus on first seen object
        y_pos_ooi = pixy.ccc.blocks[0].m_y;
        
        /// ON_BOARD BALL TRACKING MAPPING
  
        // Map ball pos to real position on ground
        y_ball_pos = 289387*pow(y_pos_ooi, -1.796);
        double xmax = y_ball_pos*widestX/farthestY;
        x_ball_pos = map(x_pos_ooi, 0, 316, -xmax, xmax);
    
        // ball position from camera is in cm due to the nature of the map function so lets make it metres to match everything else
        y_ball_pos = y_ball_pos / 100;
        x_ball_pos = x_ball_pos / 100;
    
        // Set error with target of 0
        a_error = atan(x_ball_pos/y_ball_pos);
        d_error = y_ball_pos;
        
        /// APPROACH TARGET
        // Check control feedback
        if (millis() > last_time + dt*1000) {
          a_out = PID(a_error, prev_a_error, a_integral, aKp, aKi, aKd, dt);
          baseSpeed = PID(d_error, prev_d_error, d_integral, dKp, dKi, dKd, dt);
          last_time = millis();
        }
        // Set motor speeds
        RightMotorSpeed = baseSpeed - a_out;
        LeftMotorSpeed = baseSpeed + a_out;
      break;
      case GET_BALL:
        // Drive forward at mid speed for short time;
        if (millis() < init_ball_seen_time + 600)
          state = GET_BALL;
        else
          state = CHK_DONE;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        RightMotorSpeed = 0.4*maxSpeed;
        LeftMotorSpeed = 0.4*maxSpeed;
        IntakeSpeed = 40;
      break;
      case CHK_DONE:
        if (balls = 0)
          state = GO_HOME;
        else
          state = GOTO_PT;
      break;
      case GO_HOME:
        state = WAIT;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        RightMotorSpeed = 0;
        LeftMotorSpeed = 0;
        IntakeSpeed = 0;
        // Will check if near enough to HOME and then go to WAIT
      break;
      case HOLD:
        if (command = e_stop) {
          state = HOLD;
        } else if (command = e_home) {
          state = GO_HOME;
        } else
          state = WAIT;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        RightMotorSpeed = 0;
        LeftMotorSpeed = 0;
        IntakeSpeed = 0;
      break;
      default:
        state = WAIT;
      break;
    }

    // Run Motors at whatever specified speed possibly 0
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    IMotor(conI, IntakeSpeed);

    /// PRINT Data
//    Serial.print("xscreenpos: ");
//    Serial.print(x_pos_ooi);
//    Serial.print("yscreenpos: ");
//    Serial.print(y_pos_ooi);
//    Serial.print("yballpos:  ");
//    Serial.print(y_pos);
//    Serial.print("   xballpos:  ");
//    Serial.print(x_pos);
    Serial.print("     Angle ofst from target: ");
    Serial.print(a_error);
    Serial.print("  R: ");
    Serial.print(RightMotorSpeed);
    Serial.print("   L: ");
    Serial.print(LeftMotorSpeed);
    Serial.print("   x: ");
    Serial.print(x);
    Serial.print("   y: ");
    Serial.print(y);
    Serial.print("  angle: ");
    Serial.println(a);
}

///END OF MAIN
//
//
//
///FUNCTIONS BELOW

void set_pwm_frequency(int input_frequency) {

  InitTimersSafe();
  
  bool set_timer4_success = Timer4_SetFrequency(input_frequency);
  Serial.print("  Setting timer 4 frequency: ");
  Serial.print(set_timer4_success);

  bool set_timer5_success = Timer5_SetFrequency(input_frequency);
  Serial.print("  Setting timer 5 frequency: ");
  Serial.print(set_timer5_success);
}

// PID controller
double PID(double error, double &prev_error, double &integral, double Kp, double Ki, double Kd, double dt) {
  double proportional = error;
  integral += error*dt;
  double derivative = (error - prev_error) / dt;
  prev_error = error;
  double out = Kp*proportional + Ki*integral + Kd*derivative;
  return out;
}

// Limits the maximum speed of the motors to a chosen cap
void LimitMotors(int maxSpeed) {
  
    //If the motors are > maxSpeed, or < -maxSpeed, we set speed to the abs(max)

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

/// ODOMETRY FUNCTIONS
void StartOdometry() {
  attachInterrupt(digitalPinToInterrupt(rCLK), rEncMove, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lCLK), lEncMove, CHANGE);

  rPreviousCLK = digitalRead(rCLK);
  rPreviousDATA = digitalRead(rDT);
  lPreviousCLK = digitalRead(lCLK);
  lPreviousDATA = digitalRead(lDT);

  if (!lis3mdl.begin_I2C()) {
    Serial.println("error: Could not find magnetometer");
  }
}

void rEncMove() {
  check_rotary_R();
  rPreviousCLK = digitalRead(rCLK);
  rPreviousDATA = digitalRead(rDT);
}

void lEncMove() {
  check_rotary_L();
  lPreviousCLK = digitalRead(lCLK);
  lPreviousDATA = digitalRead(lDT);
}

void Odometry() {

//COMPASS
  static float hi_cal[3];
  // Get new sensor event with readings in uTesla
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  // Put raw magnetometer readings into an array
  float mag_data[] = {event.magnetic.x,
                      event.magnetic.y,
                      event.magnetic.z};
  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +
                  (soft_iron[i][1] * hi_cal[1]) +
                  (soft_iron[i][2] * hi_cal[2]);
  }
  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  heading = -1.0 * (atan2(mag_data[0], mag_data[1])*180)/PI;
  a = heading/180*PI;
  // Convert heading to 0..360 degrees
  if (a < 0) {
    a += 2*PI;
  }
  
  if (abs(rcounter) > CLKS_PER_SAMPLE || abs(lcounter) > CLKS_PER_SAMPLE) {
    double rdistance = rcounter * DIST_PER_CLK;
    double ldistance = lcounter * DIST_PER_CLK;
    double dd = (rdistance + ldistance) / 2.0;
    double dx = cos(a) * dd;
    double dy = sin(a) * dd;
    x = x + dx;
    y = y + dy;
    
    rcounter = 0;
    lcounter = 0;
  }

}

// Check if Rotary Encoder was moved on thr right side
void check_rotary_R() {

 if ((rPreviousCLK == 0) && (rPreviousDATA == 1)) {
    if ((digitalRead(rCLK) == 1) && (digitalRead(rDT) == 0)) {
      rcounter++;
      return;
    }
    if ((digitalRead(rCLK) == 1) && (digitalRead(rDT) == 1)) {
      rcounter--;
      return;
    }
  }

if ((rPreviousCLK == 1) && (rPreviousDATA == 0)) {
    if ((digitalRead(rCLK) == 0) && (digitalRead(rDT) == 1)) {
      rcounter++;
      return;
    }
    if ((digitalRead(rCLK) == 0) && (digitalRead(rDT) == 0)) {
      rcounter--;
      return;
    }
  }

if ((rPreviousCLK == 1) && (rPreviousDATA == 1)) {
    if ((digitalRead(rCLK) == 0) && (digitalRead(rDT) == 1)) {
      rcounter++;
      return;
    }
    if ((digitalRead(rCLK) == 0) && (digitalRead(rDT) == 0)) {
      rcounter--;
      return;
    }
  }  

if ((rPreviousCLK == 0) && (rPreviousDATA == 0)) {
    if ((digitalRead(rCLK) == 1) && (digitalRead(rDT) == 0)) {
      rcounter++;
      return;
    }
    if ((digitalRead(rCLK) == 1) && (digitalRead(rDT) == 1)) {
      rcounter--;
      return;
    }
  }       
}

void check_rotary_L() {

 if ((lPreviousCLK == 0) && (lPreviousDATA == 1)) {
    if ((digitalRead(lCLK) == 1) && (digitalRead(lDT) == 0)) {
      lcounter--;
      return;
    }
    if ((digitalRead(lCLK) == 1) && (digitalRead(lDT) == 1)) {
      lcounter++;
      return;
    }
  }

if ((lPreviousCLK == 1) && (lPreviousDATA == 0)) {
    if ((digitalRead(lCLK) == 0) && (digitalRead(lDT) == 1)) {
      lcounter--;
      return;
    }
    if ((digitalRead(lCLK) == 0) && (digitalRead(lDT) == 0)) {
      lcounter++;
      return;
    }
  }

if ((lPreviousCLK == 1) && (lPreviousDATA == 1)) {
    if ((digitalRead(lCLK) == 0) && (digitalRead(lDT) == 1)) {
      lcounter--;
      return;
    }
    if ((digitalRead(lCLK) == 0) && (digitalRead(lDT) == 0)) {
      lcounter++;
      return;
    }
  }  

if ((lPreviousCLK == 0) && (lPreviousDATA == 0)) {
    if ((digitalRead(lCLK) == 1) && (digitalRead(lDT) == 0)) {
      lcounter--;
      return;
    }
    if ((digitalRead(lCLK) == 1) && (digitalRead(lDT) == 1)) {
      lcounter++;
      return;
    }
  }       
 }
