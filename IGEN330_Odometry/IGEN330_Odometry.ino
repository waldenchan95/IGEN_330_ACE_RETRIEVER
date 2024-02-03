// Encoder Read Code
// Global variables rCounter and lCounter are incremented with intterupts as the encoders move
// This code will be used by Odometry to calculate robot position

// Right encoder pins
#define R_CLK 2
#define R_DT 4

// Left encoder pins
#define L_CLK 3
#define L_DT 5

#define DISTANCE_PER_COUNT 0.018849559 // (metres) 12cm wheel with 90 encoder counts per revolution
#define BOT_WIDTH 0.4 // (metres)
#define SAMPLE_RATE 80 // (ms)

// Right Variables
int rCounter = 0;
double rDistance = 0;

// Left Variables
int lCounter = 0;
double lDistance = 0;

// Odometry Variables
double x = 0;
double dx = 0;
double y = 0;
double dy = 0;
double a = 1.570796327;
double da = 0;
double dd = 0;
unsigned long lastSample = 0;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(R_CLK),  rMove, FALLING);
  attachInterrupt(digitalPinToInterrupt(L_CLK),  lMove, FALLING);
  pinMode(R_DT,INPUT);
  pinMode(L_DT,INPUT);
}

void rMove() {  
    if (digitalRead(R_DT) == 1) {
      rCounter++;
    }
    if (digitalRead(R_DT) == 0) {
      rCounter--;  
    } 

    rDistance = rCounter * DISTANCE_PER_COUNT;

//    Serial.print("Right: ");
//    Serial.print(rCounter);
//    Serial.print(" ");
//    Serial.print("R_Distance: ");
//    Serial.print(rDistance);
//    Serial.print(" ");
//    Serial.print("Left: ");
//    Serial.print(lCounter);
//    Serial.print(" ");
//    Serial.print("L_Distance: ");
//    Serial.println(lDistance);
}

void lMove() {  
    if (digitalRead(L_DT) == 1) {
      lCounter++;
    }
    if (digitalRead(L_DT) == 0) {
      lCounter--;  
    } 

    lDistance = lCounter * DISTANCE_PER_COUNT;

//    Serial.print("Right: ");
//    Serial.print(rCounter);
//    Serial.print(" ");
//    Serial.print("R_Distance: ");
//    Serial.print(rDistance);
//    Serial.print(" ");
//    Serial.print("Left: ");
//    Serial.print(lCounter);
//    Serial.print(" ");
//    Serial.print("L_Distance: ");
//    Serial.println(lDistance);
}

void loop() {
  if (millis() - lastSample > SAMPLE_RATE) {
    lastSample = millis();
    da = atan( (rDistance - lDistance) / BOT_WIDTH);
    dd = (rDistance + lDistance) / 2.0;
    a = a + da;
    dx = cos(a) * dd;
    dy = sin(a) * dd;
    x = x + dx;
    y = y + dy;

    rDistance = 0;
    lDistance = 0;
    rCounter = 0;
    lCounter = 0;
    dd = 0;
    da = 0;
    dx = 0;
    dy = 0;

    Serial.print("  x = ");
    Serial.print(x, 4);
    Serial.print("  y = ");
    Serial.print(y, 4);
    Serial.print("  angle = ");
    Serial.println(a, 4);
  }
}
