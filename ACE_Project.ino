#include <Pixy2.h>
#include <math.h>
#include <PWM.h>
//Set pixy as main object
Pixy2 pixy;

// initialize variables

int x_position_ooi; //x position of object of interest
int y_position_ooi; //y position of object of interest

// drive variables

const int RinB = 8;
const int RinF = 7;
const int LinB = 2;
const int LinF = 3;
const int conR = 6;
const int conL = 5;

int input_frequency = 200

// control variables

int x_error = 0;
int y_error = 0;

int RightMotorSpeed;
int LeftMotorSpeed;

void set_pwm_frequency(int frequency){
  bool set_R_B_in = SetPinFrequency(RinB, frequency);
  Serial.print("Setting R_B Frequency: ");
  Serial.print(set_R_B_in);

  bool set_R_F_in = SetPinFrequency(RinF, frequency);
  Serial.print("Setting R_F Frequency: ");
  Serial.print(set_R_F_in);

  bool set_L_B_in = SetPinFrequency(LinB, frequency);
  Serial.print("Setting L_B Frequency: ");
  Serial.print(set_L_B_in);

  bool set_L_F_in = SetPinFrequency(LinF, frequency);
  Serial.print("Setting L_F Frequency: ");
  Serial.print(set_L_F_in);

  bool set_con_R_in = SetPinFrequency(conR, frequency);
  Serial.print("Setting conR Frequency: ");
  Serial.print(set_con_R_in);

  bool set_con_L_in = SetPinFrequency(conL, frequency);
  Serial.print("Setting conL Frequency: ");
  Serial.print(set_con_L_in);
}

void setup() {
  // initialize output pins
  pinMode(RinB, OUTPUT);
  pinMode(RinF, OUTPUT);
  pinMode(LinB, OUTPUT);
  pinMode(LinF, OUTPUT);

  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);
  InitTimersSafe();
  // start serial communication
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // intialize pixy library
  pixy.init();

}


// Functions
void RMotor (int speed) {
  // Forwards if input positive
  if (speed >= 0) {
    pwmWrite(RinF, HIGH);
    pwmWrite(RinB, LOW);
    pwmWrite(conR, speed);
  } else { // Backwards is input negative
    pwmWrite(RinF, LOW);
    pwmWrite(RinB, HIGH);
    pwmWrite(conR, abs(speed));
  }
}

void LMotor (int speed) {
  // Forwards if input positive
  if (speed >= 0) {
    pwmWrite(LinF, HIGH);
    pwmWrite(LinB, LOW);
    pwmWrite(conL, speed);
  } else { // Backwards is input negative
    pwmWrite(LinF, LOW);
    pwmWrite(LinB, HIGH);
    pwmWrite(conL, abs(speed));
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  int i;
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
 
  if (pixy.ccc.numBlocks)
  {
    //Serial.print("Detected ");
    //Serial.print(pixy.ccc.numBlocks);
    //Serial.println(" objects");
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
        x_position_ooi = pixy.ccc.blocks[i].m_x;
        y_position_ooi = pixy.ccc.blocks[i].m_y;
    }
  }
    set_pwm_frequency(input_frequency);
    x_error = map(x_position_ooi, 0, 316, -100, 100);
    y_error = map(y_position_ooi, 0, 316, 400, -150);

    x_error = int(pow(x_error, 3)/5000);
   
    RightMotorSpeed = y_error - x_error;
    LeftMotorSpeed = y_error + x_error;


    if(RightMotorSpeed > 255)
    {
      RightMotorSpeed = 255;
    }
   
    if(RightMotorSpeed > 255)
    {
      RightMotorSpeed = 255;
    }

   
    if(LeftMotorSpeed < -255)
    {
      LeftMotorSpeed = -255;
    }
   
    if(LeftMotorSpeed < -255)
    {
      LeftMotorSpeed = -255;
    }

    Serial.print("RightSpeed: ");
    Serial.print(RightMotorSpeed);

    Serial.print("LeftSpeed: ");
    Serial.println(LeftMotorSpeed);

    RMotor(RightMotorSpeed);
    LMotor(LeftMotorSpeed);
  }
