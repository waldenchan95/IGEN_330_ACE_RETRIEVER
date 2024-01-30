#include <Pixy2.h>

#include <math.h>

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

// control variables

int x_error = 0;
int y_error = 0;

int RightMotorSpeed;
int LeftMotorSpeed;

void setup() {
  // initialize output pins
  pinMode(RinB, OUTPUT);
  pinMode(RinF, OUTPUT);
  pinMode(LinB, OUTPUT);
  pinMode(LinF, OUTPUT);

  pinMode(conR, OUTPUT);
  pinMode(conL, OUTPUT);

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
    digitalWrite(RinF, HIGH);
    digitalWrite(RinB, LOW);
    analogWrite(conR, speed);
  } else { // Backwards is input negative
    digitalWrite(RinF, LOW);
    digitalWrite(RinB, HIGH);
    analogWrite(conR, abs(speed));
  }
}

void LMotor (int speed) {
  // Forwards if input positive
  if (speed >= 0) {
    digitalWrite(LinF, HIGH);
    digitalWrite(LinB, LOW);
    analogWrite(conL, speed);
  } else { // Backwards is input negative
    digitalWrite(LinF, LOW);
    digitalWrite(LinB, HIGH);
    analogWrite(conL, abs(speed));
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

   
    if(RightMotorSpeed < -255)
    {
      RightMotorSpeed = -255;
    }
   
    if(RightMotorSpeed < -255)
    {
      RightMotorSpeed = -255;
    }

    Serial.print("RightSpeed: ");
    Serial.print(RightMotorSpeed);

    Serial.print("LeftSpeed: ");
    Serial.println(LeftMotorSpeed);

    RMotor(RightMotorSpeed);
    LMotor(LeftMotorSpeed);
  }