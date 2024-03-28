// NEED TO ADD:
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
enum States { SETUP, WAIT, GOTO_PT, GOTO_BALL, GET_BALL, CHK_DONE, GO_HOME, HOLD } state, nxt_state;
// Commands
enum Commands { start, e_stop, e_home } command;
// Controlled externally: tells robot if there are any more balls it must go to
bool pt_valid = 0;

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
double startingAngle = 0; // This angle is whichever way the robot is facing upon startup which becomes the origin angle
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
const int baseSpeedMax = 180;
// PID
const double dt = 0.005; // (s) time between a_error updates (multiplied by 1000 to be used in millis()) 
// Angle PID constants
const double aK = 140; // MASTER GAIN rotation
const double aKi = 120; // integral multiplier
const double aKd = 1.6; // derivative multiplier
// BaseSpeed PID constants
const double dK = 60; // MASTER GAIN drive
const double dKi = 34; // integral multiplier
const double dKd = 2; // derivative multiplier

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
  state = SETUP;
}

void loop() {
    /// EXECUTE ODOMETRY
    Odometry();

    /// STATEMACHINE
    switch(state) {
      case SETUP:
        if (millis() > 1000) { // give some time for odometry to read position
          nxt_state = WAIT;
          startingAngle = a;
        }
        else
          nxt_state = SETUP;
      break;
      case WAIT:
        // Wait for signal
        if (command == start) {
          nxt_state = GOTO_PT;
          init_ball_seen_time = millis();
          prev_a_error = 0;
          prev_d_error = 0;
          a_error = 0;
          d_error = 0;
          last_time = millis();
          RightMotorSpeed = 0;
          LeftMotorSpeed = 0;
          IntakeSpeed = 0;
        }
        else if (command == e_home)
          nxt_state = GO_HOME;
        else
          nxt_state = WAIT;
      
        // Maintain starting angle.
        // This will also revert the robot back to the correct angle after coming home during the GO_HOME state
        a_error = a;
        if (millis() > last_time + dt*1000) {
          a_out = PID(a_error, prev_a_error, a_integral, aK, aKi, aKd, dt);
          last_time = millis();
        }
        
        RightMotorSpeed = -a_out;
        LeftMotorSpeed = a_out;
      break;
      case GOTO_PT:
        if (!pt_valid) {
          nxt_state = GOTO_BALL;
        }
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks && millis() > init_ball_seen_time + 500) {
            nxt_state = GOTO_BALL;
            prev_a_error = 0;
            prev_d_error = 0;
            a_error = 0;
            d_error = 0;
            last_time = millis();
            RightMotorSpeed = 0;
            LeftMotorSpeed = 0;
            IntakeSpeed = 0;
        } else {
          init_ball_seen_time = millis(); // when ball is seen this will be the initial time it was seen
          nxt_state = GOTO_PT;
        }
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        /// EXTERNAL CAMERA/POINT TARGET
        IntakeSpeed = 0; // DO NOT Run intake when ball NOT in view
              // Do things based on external camera
              // Set targets:
              // Overwrite camera-based ball positions and angle error with other points and angles for robot to go to
              // Compute angle between bot and target, target 0
              // Compute distance from point, target 0
        /// APPROACH TARGET
        // Check control feedback
        if (millis() > last_time + dt*1000) {
          a_out = PID(a_error, prev_a_error, a_integral, aK, aKi, aKd, dt);
          baseSpeed = PID(d_error, prev_d_error, d_integral, dK, dKi, dKd, dt);
          last_time = millis();
        }
        
        if (baseSpeed > baseSpeedMax) {
          baseSpeed = baseSpeedMax;
        }
        // Set motor speeds
        RightMotorSpeed = baseSpeed - a_out;
        LeftMotorSpeed = baseSpeed + a_out;
      break;
      case GOTO_BALL:
        IntakeSpeed = 40; // Run intake when ball in view
        
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks) {
          nxt_state = GOTO_BALL;
        } else {
          nxt_state = GET_BALL;
          init_ball_seen_time = millis();
        }
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
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
          a_out = PID(a_error, prev_a_error, a_integral, aK, aKi, aKd, dt);
          baseSpeed = PID(d_error, prev_d_error, d_integral, dK, dKi, dKd, dt);
          last_time = millis();
        }

        if (baseSpeed > baseSpeedMax) {
          baseSpeed = baseSpeedMax;
        }
        // Set motor speeds
        RightMotorSpeed = baseSpeed - a_out;
        LeftMotorSpeed = baseSpeed + a_out;
      break;
      case GET_BALL:
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        // Drive forward at mid speed for short time;
        if (millis() < init_ball_seen_time + 600)
          nxt_state = GET_BALL;
        else
          nxt_state = CHK_DONE;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        last_time = millis();
        RightMotorSpeed = 0.4*dK;
        LeftMotorSpeed = 0.4*dK;
        IntakeSpeed = 40;
      break;
      case CHK_DONE:
        if (pt_valid == 0)
          nxt_state = GO_HOME;
        else
          nxt_state = GOTO_PT;
      break;
      case GO_HOME:
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        nxt_state = WAIT;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        last_time = millis();
        RightMotorSpeed = 0;
        LeftMotorSpeed = 0;
        IntakeSpeed = 0;
        // will go home
        // Will check if near enough to HOME and then go to WAIT
      break;
      case HOLD:
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        } else
          nxt_state = WAIT;
        prev_a_error = 0;
        prev_d_error = 0;
        a_error = 0;
        d_error = 0;
        last_time = millis();
        RightMotorSpeed = 0;
        LeftMotorSpeed = 0;
        IntakeSpeed = 0;
      break;
      default:
        nxt_state = WAIT;
      break;
    }
    state = nxt_state;

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
//    Serial.print("     Angle ofst from target: ");
//    Serial.print(a_error);
//    Serial.print("  R: ");
//    Serial.print(RightMotorSpeed);
//    Serial.print("   L: ");
//    Serial.print(LeftMotorSpeed);
    Serial.print("   x: ");
    Serial.print(x);
    Serial.print("   y: ");
    Serial.print(y);
    Serial.print("  angle: ");
    Serial.print(a);
    Serial.print("  STATE: ");
    Serial.println(state);
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
  // Correct angle to starting position
  a = a - startingAngle;
  // Convert heading to 0-2pi degrees
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
