#include <Pixy2.h>
#include <math.h>
Pixy2 pixy;

int x_position_ooi; //x position of object of interest
int y_position_ooi; //y position of object of interest

int x_error = 0;
int y_error = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pixy.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  // grab blocks!
  pixy.ccc.getBlocks();
 
  // If there are detect blocks, print them!
 
  if (pixy.ccc.numBlocks)
  {
    //Serial.print("Detected ");
    //Serial.print(pixy.ccc.numBlocks);
    //Serial.println(" objects");
    for (int i=0; i<pixy.ccc.numBlocks; i++)
    {
        x_position_ooi = pixy.ccc.blocks[i].m_x;
        y_position_ooi = pixy.ccc.blocks[i].m_y;
    }
  }
 
    x_error = map(x_position_ooi, 0, 316, -100, 100);
    y_error = map(y_position_ooi, 0, 316, 400, -150);

    x_error = int(pow(x_error, 3)/5000);
    Serial.print("X_error: ");
    Serial.print(x_error);
    Serial.print("  Y_error: ");
    Serial.println(y_error);
}
