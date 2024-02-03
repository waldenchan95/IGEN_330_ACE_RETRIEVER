// Odometry Header File
// Include in main file and follow instruction for creating of variables and defining constants
// User will have constantly updated x, y, and a (angle) position data

// Right encoder pins
#define R_CLK 2
#define R_DT 4

// Left encoder pins
#define L_CLK 3
#define L_DT 5

// Constants
// define in main code:
// botWidth
// dPerClk
#define SAMPLE_RATE 80 // (ms)

// More:
// create variables 
// double x = 0
// double y = 0
// double a = 1.570796327 (depends on coordinate system)
// unsigned long lastEncoderSample = 0
// int rCounter = 0
// int lCounter = 0

// Interrupt initialization
void StartOdometry() {
  attachInterrupt(digitalPinToInterrupt(R_CLK),  rMove, FALLING);
  attachInterrupt(digitalPinToInterrupt(L_CLK),  lMove, FALLING);
  pinMode(R_DT,INPUT);
  pinMode(L_DT,INPUT);
}

// ISR's
void rMove() {  
    if (digitalRead(R_DT) == 1) {
      rCounter++;
    }
    if (digitalRead(R_DT) == 0) {
      rCounter--;  
    } 
}

void lMove() {  
    if (digitalRead(L_DT) == 1) {
      lCounter++;
    }
    if (digitalRead(L_DT) == 0) {
      lCounter--;  
    } 
}

// Odometry
void Odometry() {
  if (millis() - lastEncoderSample > SAMPLE_RATE) {
    lastSample = millis();
    double da = atan( (rDistance - lDistance) / BOT_WIDTH);
    double dd = (rDistance + lDistance) / 2.0;
    a = a + da;
    double dx = cos(a) * dd;
    double dy = sin(a) * dd;
    x = x + dx;
    y = y + dy;
    
    rCounter = 0;
    lCounter = 0;

    Serial.print("  x = ");
    Serial.print(x, 4);
    Serial.print("  y = ");
    Serial.print(y, 4);
    Serial.print("  angle = ");
    Serial.println(a, 4);
  }
}