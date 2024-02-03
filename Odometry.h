// Odometry Header File
// Include in main file
// User will have constantly updated x, y, and a (angle) position data

// Right encoder pins
#define R_CLK 2
#define R_DT 4

// Left encoder pins
#define L_CLK 3
#define L_DT 5

// Constants
#define SAMPLE_RATE 80 // (ms)

extern const double botWidth = 0.4; // m
extern const double dPerClk = 0.01;


extern double x = 0;
extern double y = 0;
extern double a = 1.570796327;
extern unsigned long lastEncoderSample = 0;
extern int rCounter = 0;
extern int lCounter = 0;

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

// Interrupt initialization
void StartOdometry() {
  attachInterrupt(digitalPinToInterrupt(R_CLK),  rMove, FALLING);
  attachInterrupt(digitalPinToInterrupt(L_CLK),  lMove, FALLING);
  pinMode(R_DT,INPUT);
  pinMode(L_DT,INPUT);
}

// Odometry
void Odometry() {

  double rDistance = rCounter * dPerClk;
  double lDistance = lCounter * dPerClk;
  
  if (millis() - lastEncoderSample > SAMPLE_RATE) {
    lastEncoderSample = millis();
    double da = atan( (rDistance - lDistance) / botWidth);
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
