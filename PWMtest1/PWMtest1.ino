//PWM test 1

#include <math.h>
#include <PWM.h>
#include "/Users/lucagarner/Coding/IGEN 330/IGEN_330_ACE_RETRIEVER/MotorControls/MotorControls.ino"

// drive variables
const int conR = 10;
const int conL = 9;

int input_frequency = 150;

// control variables

int RightMotorSpeed;
int LeftMotorSpeed;

void setup() {
  // initialize output pins
  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  InitTimersSafe();

  
  // start serial communication
  Serial.begin(9600);
  set_pwm_frequency(conL, input_frequency);

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(200);

  for (int i = 0; i < 100; i += 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    delay(18);
  }
  for (int i = 100; i > -100; i -= 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    delay(18);
  }
  for (int i = -100; i <= 0; i += 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(conR, RightMotorSpeed);
    LMotor(conL, LeftMotorSpeed);
    delay(18);
  }
}
