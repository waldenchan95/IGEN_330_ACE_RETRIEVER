//PWM test 1

#include <math.h>
#include "MotorControls.h"

// drive variables
const int conI = 44;
const int conR = 46;
const int conL = 45;

int input_frequency = 150;

void setup() {
  // initialize output pins
  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  pinMode(conI, OUTPUT);
  InitTimersSafe();

  // start serial communication
  Serial.begin(9600);
  set_pwm_frequency(input_frequency);
}

void loop() {

    IMotor(conI, -30);
    RMotor(conR, 100);
    LMotor(conL, 100);
    delay(300);
    RMotor(conR, 200);
    LMotor(conL, 200);
    delay(7000);
  
}
