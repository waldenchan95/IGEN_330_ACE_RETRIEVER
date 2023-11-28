// Encoder Read Code
// Global variables rCounter and lCounter are incremented with intterupts as the encoders move
// This code will be used by Odometry to calculate robot position


#include "PinChangeInterrupt.h"

// Right encoder pins
#define R_CLK 2
#define R_DT 3

// Left encoder pins
#define L_CLK 7
#define L_DT 8

// Right Variables
int rCounter = 0;
int rCurrentState;
int rInitState;

// Left Variables
int lCounter = 0;
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
  attachInterrupt(digitalPinToPCINT(R_CLK), RightEncoderCount, CHANGE);
  attachInterrupt(digitalPinToPCINT(L_DT), RightEncoderCount, CHANGE);

  // attach left encoder interrupts
  attachInterrupt(digitalPinToPCINT(L_CLK), LeftEncoderCount, CHANGE);
  attachInterrupt(digitalPinToPCINT(L_DT), LeftEncoderCount, CHANGE);
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
    
    // print the value in the serial monitor window
    Serial.print("Right: ");
    Serial.print(rCounter);
    Serial.print("Left: ");
    Serial.print(lCounter);
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
    
    // print the value in the serial monitor window
    Serial.print("Right: ");
    Serial.print(rCounter);
    Serial.print("Left: ");
    Serial.print(lCounter);
  }

  // Remember last CLK state for next cycle
  lInitState = lCurrentState;
}
