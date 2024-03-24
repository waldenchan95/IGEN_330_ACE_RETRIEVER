#include <PWM.h>

#define PWM_LOW 95
#define PWM_HIGH 20


// Motor Functions:
// Takes -255 to 255 inputs and maps them to correct PWM for the robots motors
void RMotor (int conR, int speed) {
  int adjSpeed = map(speed, 255, -255, PWM_LOW, PWM_HIGH);
  pwmWrite(conR, adjSpeed);
}

void LMotor (int conL, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conL, adjSpeed);
}

void IMotor (int conI, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conI, adjSpeed);
}
