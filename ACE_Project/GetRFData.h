list_of_points = [[]]

void setup() 
{
  pinMode(13,OUTPUT); 
  Serial.begin(9600);
}


void loop() 
{
  if(Serial.available() > 0)
  {
    list_of_points = Serial.read()
  }  

}
