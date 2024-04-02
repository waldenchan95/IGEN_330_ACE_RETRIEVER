#include <PWM.h>

#define PWM_LOW 8000
#define PWM_HIGH 16000


// Motor Functions:
// Takes -255 to 255 inputs and maps them to correct PWM for the robots motors
void RMotor (int conR, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  analogWrite(conR, adjSpeed);
}

void LMotor (int conL, int speed) {
  int adjSpeed = map(speed, 255, -255, PWM_LOW, PWM_HIGH);
  analogWrite(conL, adjSpeed);
}

void IMotor (int conI, int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
 analogWrite(conI, adjSpeed);
}

//// Limits the maximum speed of the motors to a chosen cap
//void LimitMotors(int maxSpeed) {
//  
//    //If the motors are > maxSpeed, or < -maxSpeed, we set speed to the abs(max)
//
//    // Right
//    if(RightMotorSpeed > maxSpeed)
//    {
//      RightMotorSpeed = maxSpeed;
//    }
//   
//    if(RightMotorSpeed > maxSpeed)
//    {
//      RightMotorSpeed = maxSpeed;
//    }
//
//    if(RightMotorSpeed < -maxSpeed)
//    {
//      RightMotorSpeed = -maxSpeed;
//    }
//   
//    if(RightMotorSpeed < -maxSpeed)
//    {
//      RightMotorSpeed = -maxSpeed;
//    }
//
//    // Left
//    if(LeftMotorSpeed > maxSpeed)
//    {
//      LeftMotorSpeed = maxSpeed;
//    }
//   
//    if(LeftMotorSpeed > maxSpeed)
//    {
//      LeftMotorSpeed = maxSpeed;
//    }
//
//    if(LeftMotorSpeed < -maxSpeed)
//    {
//      LeftMotorSpeed = -maxSpeed;
//    }
//   
//    if(LeftMotorSpeed < -maxSpeed)
//    {
//      LeftMotorSpeed = -maxSpeed;
//    }
//}
