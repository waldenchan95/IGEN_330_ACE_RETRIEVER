#include <math.h>
#include "Odometry.h"

//ENCODER
// Rotary Encoder Module connections
const int rDT = 7;    // DATA signal
const int rCLK = 19;    // CLOCK signal
const int lDT = 6;    // DATA signal
const int lCLK = 18;    // CLOCK signal


//Create instance of magnetometer
Adafruit_LIS3MDL lis3mdl;

void setup() {
  Serial.begin(115200);
  StartOdometry();
}

void loop() {
  Odometry();
//PRINT
  Serial.print("X:  ");
  Serial.print(x, 4);
  Serial.print("  Y:  ");
  Serial.print(y, 4);
  Serial.print(" A: ");
  Serial.println(a, 4);
}
