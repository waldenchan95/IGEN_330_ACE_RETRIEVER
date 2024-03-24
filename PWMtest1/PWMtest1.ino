//PWM test 1

#include <math.h>
#include "MotorControls.h"

// drive variables
const int conR = 46;
const int conL = 44;
const int conI = 45;

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
    IMotor(conI, 70);
}

// Changes PWM frequency for a given timer
void set_pwm_frequency(int frequency) {

  Timer5_Initialize();
  bool set_timer5_success = Timer5_SetFrequency(frequency);
  Serial.print("  Setting timer 5 frequency: ");
  Serial.print(set_timer5_success);

}
