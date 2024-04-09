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
enum States { SETUP, ANGLE_INIT_DELAY, WAIT, GOTO_PT, SCAN, GOTO_BALL, GET_BALL, GO_HOME, HOLD } state, nxt_state;
// Commands
enum Commands { wait, start, e_stop, e_home } command;
// Controlled externally: tells robot if there are any more balls it must go to
bool pt_valid = 0;

// Variables
unsigned long init_ball_seen_time; // When robot sees ball in pixy it waits a short time to determine if the object is consistent before going
unsigned long init_get_ball; // used for get ball timer
unsigned long init_ball_lost_time; // Same but for losing the ball in camera
unsigned long init_scan_time; // used for scan state timer

// Odometry Definitions
#define CLKS_PER_SAMPLE 1 // pure counts of encoder
#define DIST_PER_CLK 0.005984734*240/207 // calibrated real world (m)

// PINS
// Motor
const int conR = 6; // all should be on timer 5 of the mega
const int conL = 7; // Yellow
const int conI = 44; // 
// Rotary Encoder Module connections
const int rDT = 5;    // DATA signal  // ORnge
const int rCLK = 19;    // CLOCK signal //green
const int lDT = 4;    // DATA signal // ornGE
const int lCLK = 18;    // CLOCK signal //green

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
double a = 0; //(rad) // raw angle data
double r = 0; //(rad) angle after it has been filtered
double a_c = 0; 
double a_deg = 0; //(deg) Used purely for troubleshooting
enum DIR {E, W} dir_flg;
// Magnetometer
double startingAngle = 0; // This angle is whichever way the robot is facing upon startup which becomes the origin angle
unsigned long startingAngle_init_time = 0;
// WSF Filter
double flt_coeff[15] { 0.252, 0.1894, 0.1423, 0.1069, 0.0804, 0.0604, 0.0454, 0.0341, 0.0256, 0.0193, 0.0145, 0.0109, 0.0082, 0.0061, 0.0046 };
double flt_prev[14] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long flt_last_time = 0;
// Hard-iron calibration settings
//Mag: (39.33, 1.70, -36.63) Hard offset: (1.93, -7.37, -32.25) Field: (42.62, 37.29, 44.31)
const float hard_iron[3] = {
  1.93,  -7.37,  -30.25
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
// Ball in Camera Positional variables (Mapped to real world position)
double x_ball_pos = 0;
double y_ball_pos = 0;
// Target Point (populated by external system)
double x_target = 2; // W/O external input we can hardcode a position and try to go there
double y_target = 0;
double x_error; // components of positional error only used in intermediate steps
double y_error;
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
const long farthestY = 162; // [cm] Farthest distance ball is in camera view
const long widestX = 126; // [cm] When ball is at farthest y, the widest x can be from center
// Algorithm constants
const int baseSpeedMax = 180;
// PID
const double PIDdt = 10; // (ms) time between a_error updates (multiplied by 1000 to be used in millis()) 
// Camera Angle PID constants
const double aK = 360; // MASTER GAIN rotation (im pretrty sure these arent actually master gains theyre just Kp)
const double aKi = 45; // integral multiplier
const double aKd = 50; // derivative multiplier
// Compass Angle PID constants
//const double cK = 75; // MASTER GAIN rotation
//const double cK = 50; // MASTER GAIN rotation
//const double cKi = 32; // integral multiplier
//const double cKd = 11; // derivative multiplier
const double cK = 66; // MASTER GAIN rotation
const double cKi = 11; // integral multiplier
const double cKd = 14; // derivative multiplier
// BaseSpeed PID constant
const double dK = 86; // MASTER GAIN drive
const double dKi = 50; // integral multiplier
const double dKd = 10; // derivative multiplier

/// LOW_PASS FILTER CLASS
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

/// END OF LOW_PASS FILTER CLASS

//Create instance of magnetometer
Adafruit_LIS3MDL lis3mdl;

// Filter instance
LowPass<2> lp_compass(15,1e3,true);

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
  
  // ROBOT MODE (use without external input to change what robot does)
  state = GOTO_BALL;
  //command = wait;
  command = start;
  x_target = 7;
  y_target = 9;
  pt_valid = 1;
}

void loop() {
    /// EXECUTE ODOMETRY
    Odometry();

    /// STATEMACHINE
    switch(state) {
      case SETUP:
        if (millis() > 300) { // give some time for odometry to read position
          nxt_state = ANGLE_INIT_DELAY;
          startingAngle = r;
          Serial.print(" StartingAngle");
          Serial.println(startingAngle);
          IntakeSpeed = 0;
          startingAngle_init_time = millis();
        }
        else {
          nxt_state = SETUP;
        }
      break;
      case ANGLE_INIT_DELAY:
        if(millis() < startingAngle_init_time + 300){
            nxt_state = ANGLE_INIT_DELAY;
        } else {
          nxt_state = WAIT;
        }
        last_time = millis();
      break;
      case WAIT:
        // Wait for signal
        if (command == start) {
          nxt_state = GOTO_PT;
          init_ball_seen_time = millis();
          last_time = millis();
          RightMotorSpeed = 0;
          LeftMotorSpeed = 0;
          IntakeSpeed = 0;
        }
        
        // emergency
        else if (command == e_home)
          nxt_state = GO_HOME;
        else
          nxt_state = WAIT;
        
        // Maintain starting angle.
        // This will also revert the robot back to the correct angle after coming home during the GO_HOME state
       // a_error = startingAngle - r;
       a_error = GetAngleError(startingAngle);
       d_error = 0;
      break;
      case GOTO_PT:   
        /// EXTERNAL CAMERA/POINT TARGET
        IntakeSpeed = 0; // DO NOT Run intake when ball NOT in view

        x_error = x_target - x;
        y_error = y_target - y;
        d_error = sqrt(pow(x_error, 2) + pow(y_error, 2));
        a_error = GetAngleError(atan2(y_error, x_error));
        //Serial.print();

        if (millis() < 2000) {
          d_error = 0;
          a_error = 0;
        }
        
        // State logic
        if (!pt_valid) {
          nxt_state = GOTO_BALL;
        }
        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks && millis() > init_ball_seen_time + 500) {
            nxt_state = GOTO_BALL;
            init_ball_lost_time = millis();
            prev_a_error = 0;
            prev_d_error = 0;
            a_error = 0;
            d_error = 0;
            last_time = millis();
            IntakeSpeed = 0;
        } else if(abs(x_error) < 0.40 && abs(y_error) < 0.40) {
          nxt_state = SCAN;
          a_integral = 0;
          d_integral = 0;
          init_scan_time = millis();
          init_ball_seen_time = millis();

          // this was added for testing so it stops when near a point when we lock it in this state but shouldnt affect anything
          prev_a_error = 0;
          a_error = 0;
          prev_d_error = 0;
          d_error = 0;
          
        } else {
          init_ball_seen_time = millis(); // when ball is seen this will be the initial time it was seen
          nxt_state = GOTO_PT;
        }
        // emergency
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        //nxt_state = GOTO_PT; //Add this to stay locked in this state for testing
      break;
      case SCAN:
        // spins in circle to look for ball
        d_error = 0;
        a_error = 0;
        //IntakeSpeed = -50; //scan state indicator

        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks && millis() > init_ball_seen_time + 500) {
            nxt_state = GOTO_BALL;
            init_ball_lost_time = millis();
            prev_a_error = 0;
            prev_d_error = 0;
            a_error = 0;
            d_error = 0;
            last_time = millis();
            IntakeSpeed = 0;
      //  if (pixy.ccc.numBlocks*0) {
      //    nxt_state = GOTO_BALL;
        //} else if (millis() > init_scan_time + 2500) {
        //  nxt_state = GO_HOME;
        } else {
          nxt_state = SCAN;
        }
      break;
      case GOTO_BALL:
        IntakeSpeed = 60; // Run intake when ball in view
        
        pixy.ccc.getBlocks();
        
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
    
        if (!pixy.ccc.numBlocks) {
          a_error = 0;
          d_error = 0;
          prev_a_error = 0;
          prev_d_error = 0;
        } else {
            // Set error with target of 0
          d_error = y_ball_pos;
          double temp_a_error = atan(x_ball_pos/y_ball_pos);
          a_error = temp_a_error;
        }

        if (!pixy.ccc.numBlocks && millis() > init_ball_lost_time + 500) {
          if (abs(x_ball_pos) < 0.155 && y_ball_pos < 0.28) { // If ball lost near intake drive forward to get ball, if far scan for ball
            nxt_state = GET_BALL;
            init_get_ball = millis();
          } else {
            nxt_state = SCAN;
            init_scan_time = millis();
          }
        } else {
          nxt_state = GOTO_BALL;
          init_ball_lost_time = millis();
        }
        
        // emergency commands
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        
        nxt_state = GOTO_BALL; // for testing
      break;
      case GET_BALL:
        // emergency
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        }
        // Drive forward at mid speed for short time;
        if (millis() < init_get_ball + 600)
          nxt_state = GET_BALL;
        else
          nxt_state = GO_HOME;
          
        prev_a_error = 0;
        prev_d_error = 0.5;
        a_error = 0;
        d_error = 0.5; // d_error makes robot drive forward
        IntakeSpeed = 60;
      break;
      case GO_HOME:
        if (command == e_stop) {
          nxt_state = HOLD;
        } else if (command == e_home) {
          nxt_state = GO_HOME;
        } else if (x_error < 0.30 && y_error < 0.30) {
          nxt_state = WAIT;
        }
        
        // Set target as 0, 0 and go home
        x_error = 0 - x;
        y_error = 0 - y;
        d_error = sqrt(pow(x_error, 2) + pow(y_error, 2));
        a_error = GetAngleError(atan2(y_error, x_error));
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
        IntakeSpeed = 0;
        a_integral = 0;
        d_integral = 0;
      break;
      default:
        nxt_state = WAIT;
      break;
    }
    state = nxt_state;
    
    /// PID TO TARGET and SET MOTOR SPEEDS
    if (millis() > last_time + PIDdt) {
      if(state == GOTO_BALL) {
        a_out = PID(a_error, prev_a_error, a_integral, aK, aKi, aKd, PIDdt/1000);
      } else {
        a_out = PID(a_error, prev_a_error, a_integral, cK, cKi, cKd, PIDdt/1000);
      }
      
      baseSpeed = PID(d_error, prev_d_error, d_integral, dK, dKi, dKd, PIDdt/1000);
      last_time = millis();
    } 

    // WSF Filter Signal
    double a_out_filtered;
    if (millis() > flt_last_time + 1) {
      flt_last_time = millis();
      
      a_out_filtered = a_out*flt_coeff[0];
      for (int i = 0; i < 14; i++) {
        a_out_filtered += flt_prev[i]*flt_coeff[i+1];
      }
      for (int i = 13; i > 0; i--) {
        flt_prev[i] = flt_prev[i-1];
      }
      flt_prev[0] = a_out;
    }
    
    if (baseSpeed > baseSpeedMax) {
      baseSpeed = baseSpeedMax;
    }
    
    if (baseSpeed > 70) {
      baseSpeed = 70; //for testing
    }
    // Set motor speeds
    if (state == GOTO_BALL && !pixy.ccc.numBlocks) {
      RightMotorSpeed = 0;
      LeftMotorSpeed = 0;
    }
    if (state == HOLD) {
      RightMotorSpeed = 0;
      LeftMotorSpeed = 0;
    }
    
    if (state == SCAN) {
      RightMotorSpeed = 50;
      LeftMotorSpeed = -45;
    }
else {
    RightMotorSpeed = baseSpeed - a_out_filtered;
    LeftMotorSpeed = baseSpeed + a_out_filtered;
}
    
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    IMotor(conI, IntakeSpeed);


 
    /// PRINT Data
//    Serial.print("xscreenpos: ");
//    Serial.print(x_pos_ooi);
//    Serial.print("yscreenpos: ");
//    Serial.print(y_pos_ooi);
    Serial.print("  yballpos:  ");
    Serial.print(y_ball_pos);
    Serial.print("   xballpos:  ");
    Serial.print(x_ball_pos);
//    Serial.print("     Angle ofst from target: ");
//    Serial.print(a_error);
//    Serial.print("  R: ");
//    Serial.print(RightMotorSpeed);
//    Serial.print("   L: ");
//    Serial.print(LeftMotorSpeed);
//    Serial.print("   x: ");
//    Serial.print(x, 2);
//    Serial.print("   y: ");
//    Serial.print(y, 2);
//    Serial.print("  angle_deg: ");
//    Serial.print(a_deg, 5);
//    Serial.print("  bot_angle: ");
//    Serial.print(a, 5);
//      Serial.print("  Filtered angle: ");
//      Serial.print(r, 5);
//      Serial.print("  FLAG: ");
//      Serial.print(dir_flg);
//      Serial.print("d error:  ");
//      Serial.print(d_error);
//      Serial.print("  BASESPEED  ");
//      Serial.print(baseSpeed);
//      Serial.print("   Angle error:  ");
//      Serial.print(a_error);
//      Serial.print("  A out:  ");
//      Serial.print(a_out_filtered);
    Serial.print("  STATE: ");
    Serial.print(state);
    Serial.println();
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
double PID(double error, double &prev_error, double &integral, double Kp, double Ki, double Kd, double PIDdt) {
  double proportional = error;
  integral += error*PIDdt;
  double derivative = (error - prev_error) / PIDdt;
  prev_error = error;
  double out = Kp*proportional + Ki*integral + Kd*derivative;
  return out;
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

// Gets optimal angle error for points in space 
double GetAngleError(double t) {
  double error;
  if (dir_flg == W){
    if (r-t <= PI){
      error = r - t;
    }
    else {
      error = -2*PI - t + r;
    }
  }
  else if (dir_flg == E){
    if (r+t <= PI){
      error = -(r + t);
    }
    else {
      error = 2*PI -(t + r);
    }
  }
  return error;
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
//  a_deg = heading; // Saving the magnetic heading in degrees for troubleshooting
//  if (a_deg < 0) {
//    a_deg += 360;
//  }
  
  a = heading*PI/180;

  // Rid of discontintuity
  if (a < 0) {
    a_c = -a;
    dir_flg = W;
  }else {
    a_c = a;
    dir_flg = E;
  }

  // Compute the filtered signal
  r = lp_compass.filt(a_c);  

  double r_reversed;

  if (dir_flg == E) {
    r_reversed = -r;
  } else
    r_reversed = r;
  
  if (abs(rcounter) > CLKS_PER_SAMPLE || abs(lcounter) > CLKS_PER_SAMPLE) {
    double rdistance = rcounter * DIST_PER_CLK;
    double ldistance = lcounter * DIST_PER_CLK;
    double dd = (rdistance + ldistance) / 2.0;
    double dx = cos(r_reversed) * dd;
    double dy = sin(r_reversed) * dd;
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
