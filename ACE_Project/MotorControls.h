#include <PWM.h>

#define PWM_LOW 95
#define PWM_HIGH 20


// Motor Functions:
// Takes -255 to 255 inputs and maps them to correct PWM for the robots motors
void RMotor (int conR, int speed) {
  int adjSpeed = map(speed, 255, -255, PWM_LOW, PWM_HIGH);
  pwmWrite(conR, adjSpeed);
//  Serial.print("RightSpeed: ");
//  Serial.print(speed);
//  Serial.print("   Right_realspeed: ");
//  Serial.println(adjSpeed);
}

void LMotor (int conL, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conL, adjSpeed);
//  Serial.print("  LeftSpeed: ");
//  Serial.print(speed);
//  Serial.print("   Left_realspeed: ");
//  Serial.println(adjSpeed);
}

void IMotor (int conI, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conI, adjSpeed);
//  Serial.print("  IntakeSpeed: ");
//  Serial.print(speed);
//  Serial.print("   Intake_realspeed: ");
//  Serial.println(adjSpeed);
}
