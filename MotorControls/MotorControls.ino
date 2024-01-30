
#include <PWM.h>

void set_pwm_frequency(int frequency) {

  bool set_con_L_in = SetPinFrequencySafe(conL, frequency);
  Serial.print("  Setting conL Frequency: ");
  Serial.print(set_con_L_in);
}

void setup() {
  // put your setup code here, to run once:

}
// Motor Functions:
// Takes -255 to 255 inputs and maps them to correct PWM for the robots motors
void RMotor (int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conR, adjSpeed);
  Serial.print("RightSpeed: ");
  Serial.print(Speed);
  Serial.print("   Right_realspeed: ");
  Serial.println(adjSpeed);
}

void LMotor (int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conL, adjSpeed);
  Serial.print("  LeftSpeed: ");
  Serial.print(Speed);
  Serial.print("   Left_realspeed: ");
  Serial.println(adjSpeed);
}
void loop() {
  // put your main code here, to run repeatedly:

}
