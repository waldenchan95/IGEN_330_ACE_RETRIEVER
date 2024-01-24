// Odometry code. This code tracks the x, y position and angle of the robot based on encoders at each wheel
// Encoder read code is integrated into this file
// x, y and theta are global variables which will hold the position

// NOT YET WORKING

#include "PinChangeInterrupt.h"

// Right encoder pins
#define R_CLK 9
#define R_DT 10

// Left encoder pins
#define L_CLK 11
#define L_DT 12

#define DISTANCE_PER_COUNT 0.018849559 // (metres) 12cm wheel with 20 encoder counts per revolution

// Odometer variables
double x = 0;
double y = 0;
double theta = 0;

// Encoder variables
// Right Variables
int rCounter = 0;
double rDistance = 0;
int rCurrentState;
int rInitState;

// Left Variables
int lCounter = 0;
double lDistance = 0;
int lCurrentState;
int lInitState;

void setup() {

  pinMode(R_CLK, INPUT);
  pinMode(R_DT, INPUT);
  pinMode(L_CLK, INPUT);
  pinMode(L_DT, INPUT);

  // Setup Serial Monitor
  Serial.begin(9600);

  // Read the initial state of CLKs
  rInitState = digitalRead(R_CLK);
  lInitState = digitalRead(L_CLK);

  // attach right encoder interrupts
  attachPCINT(digitalPinToPCINT(R_CLK), RightEncoderCount, CHANGE);
  attachPCINT(digitalPinToPCINT(L_DT), RightEncoderCount, CHANGE);

  // attach left encoder interrupts
  attachPCINT(digitalPinToPCINT(L_CLK), LeftEncoderCount, CHANGE);
  attachPCINT(digitalPinToPCINT(L_DT), LeftEncoderCount, CHANGE);
}

void loop() {

}

void RightEncoderCount() {

  // Read the current state of CLK
  rCurrentState = digitalRead(R_CLK);
  // If last and current state of CLK are different, then we can be sure that the pulse occurred
  if (rCurrentState != rInitState  && rCurrentState == 1) {
    // Encoder is rotating counterclockwise so we decrement the counter
    if (digitalRead(R_DT) != rCurrentState) {
      rCounter++;
    } else {
      // Encoder is rotating clockwise so we increment the counter
      rCounter--;
    }

    rDistance = rCounter * DISTANCE_PER_COUNT;

  }

  // Remember last CLK state for next cycle
  rInitState = rCurrentState;
}

void LeftEncoderCount() {

  // Read the current state of CLK
  lCurrentState = digitalRead(L_CLK);
  // If last and current state of CLK are different, then we can be sure that the pulse occurred
  if (lCurrentState != lInitState  && lCurrentState == 1) {
    // Encoder is rotating counterclockwise so we decrement the counter
    if (digitalRead(L_DT) != lCurrentState) {
      lCounter++;
    } else {
      // Encoder is rotating clockwise so we increment the counter
      lCounter--;
    }

    lDistance = lCounter * DISTANCE_PER_COUNT;
    
  }

  // Remember last CLK state for next cycle
  lInitState = lCurrentState;
}
