//PWM test 1
// Dated file. we didnt actua;lly use this to test
#include <math.h>
#include "MotorControls.h"

// drive variables
const int conR = 6;
const int conL = 44;
const int conI = 45;

int input_frequency = 150;

void setup() {
  // initialize output pins
  InitTimersSafe();

  // start serial communication
  Serial.begin(9600);
  set_pwm_frequency(input_frequency);
}

void loop() {
    RMotor(conR, 30);
}

// Changes PWM frequency for a given timer
void set_pwm_frequency(int input_frequency) {

  Timer4_Initialize();
  bool set_timer4_success = Timer4_SetFrequency(input_frequency);
  Serial.print("  Setting timer 4 frequency: ");
  Serial.print(set_timer4_success);

}
