//PWM test 1

#include <math.h>
#include <PWM.h>

// PWM duty cycle values for input to motor controllers
#define PWM_LOW 95
#define PWM_HIGH 20

// initialize variables

int x_position_ooi; //x position of object of interest
int y_position_ooi; //y position of object of interest

// drive variables
const int conR = 10;
const int conL = 9;

int input_frequency = 150;

// control variables

int x_error = 0;
int y_error = 0;

int RightMotorSpeed;
int LeftMotorSpeed;

void set_pwm_frequency(int frequency) {

  bool set_con_L_in = SetPinFrequencySafe(conL, frequency);
  Serial.print("  Setting conL Frequency: ");
  Serial.print(set_con_L_in);
}

void setup() {
  // initialize output pins
  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  InitTimersSafe();

  
  // start serial communication
  Serial.begin(9600);
  set_pwm_frequency(input_frequency);

}

// Motor Functions:
// Takes -255 to 255 inputs and maps them to correct PWM for the robots motors
void RMotor (int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conR, adjSpeed);
  Serial.print("RightSpeed: ");
  Serial.print(speed);
  Serial.print("   Right_realspeed: ");
  Serial.println(adjSpeed);
}

void LMotor (int speed) {
  int adjSpeed = map(speed, -255, 255, PWM_LOW, PWM_HIGH);
  pwmWrite(conL, adjSpeed);
  Serial.print("  LeftSpeed: ");
  Serial.print(speed);
  Serial.print("   Left_realspeed: ");
  Serial.println(adjSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(200);

  RMotor(

  for (int i = 0; i < 100; i += 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(RightMotorSpeed);
    LMotor(LeftMotorSpeed);
    delay(18);
  }
  for (int i = 100; i > -100; i -= 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(RightMotorSpeed);
    LMotor(LeftMotorSpeed);
    delay(18);
  }
  for (int i = -100; i <= 0; i += 3) {
    RightMotorSpeed = i;
    LeftMotorSpeed = i;
  
    RMotor(RightMotorSpeed);
    LMotor(LeftMotorSpeed);
    delay(18);
  }
}
