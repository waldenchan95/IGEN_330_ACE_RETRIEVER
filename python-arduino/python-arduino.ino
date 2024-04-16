#include "SerialTransfer.h"
#include <string.h>

SerialTransfer myTransfer;

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10
#define BUTTON_PIN 3

RF24 radio(CE_PIN, CSN_PIN);

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
int xAnalogValue, yAnalogValue;  // Joystick values
bool buttonState;
bool newNodeRequest = false; // Variable to indicate new node request

struct __attribute__((packed)) STRUCT1 {
  double x;
  double y;
  } recieveStruct;

struct __attribute__((packed)) STRUCT3 {
  int coords[2];
  } sendListStruct;
  
  struct __attribute__((packed)) STRUCT2 {
  double x;
  double y;
  } sendStruct;

struct JoystickData 
  {
  int16_t xValue; // Use int16_t for a 16-bit signed integer
  int16_t yValue;
  bool buttonState;
  };

JoystickData dataToSend;
unsigned long currentMillis;
unsigned long prevMillis = 0;
const long txIntervalMillis = 50;  // Set your desired interval for sending data



void setup()
{
  Serial.begin(115200);
  myTransfer.begin(Serial);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(thisSlaveAddress);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  sendStruct.x = 5;
  sendStruct.y = 5;
  sendListStruct.coords[0] = 5;
  sendListStruct.coords[1] = 5;
}


void loop()
{
  
  //process_python_data_list();
  // xAnalogValue = analogRead(A0);
  // yAnalogValue = analogRead(A1);
  // buttonState = digitalRead(BUTTON_PIN);

  //Serial.print("X Analog Pin: ");
  //Serial.print(xAnalogValue);
  //Serial.print(" Y Analog Pin: ");
  //Serial.print(yAnalogValue);
  //Serial.print(" Button State: ");
  //Serial.println(buttonState);

  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    // dataToSend.xValue = xAnalogValue;
    // dataToSend.yValue = yAnalogValue;
    // dataToSend.buttonState = buttonState;
    // send_joystick_commands();
    send_coords();
    prevMillis = millis();
  }
}


void send_joystick_commands() {
  bool rslt;
  rslt = radio.write(&dataToSend, sizeof(dataToSend));

  Serial.print("Data Sent - X: ");
  Serial.print(dataToSend.xValue);
  Serial.print(", Y: ");
  Serial.print(dataToSend.yValue);
  Serial.print(", Button: ");
  Serial.print(dataToSend.buttonState);

  if (rslt) {
    Serial.println("  Acknowledge received");
    updateMessage();
  } else {
    Serial.println("  Tx failed");
  }
}

void send_coords() {
  int arr[4];
  if(myTransfer.available())
  {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(arr, recSize);
  }
  delay(500);
  uint32_t sendSize = 0;
  uint32_t pass_back[4] = {1, 1, 1, 1};
  ///////////Stuff buffer with array
  sendSize = myTransfer.txObj(pass_back, sendSize);

  ///////////Send back the recieved array to Python for debugging
  myTransfer.sendData(sendSize);
  
  bool rslt;
  rslt = radio.write(&arr, sizeof(arr));

  if (rslt) {
    Serial.println("  Acknowledge received");
    updateMessage();
  } else {
    Serial.println("  Tx failed");
  }
}

void updateMessage() {
  // Increment a counter or make any changes to show new data is being sent
}

void checkForNewNodeRequest() {
  unsigned long startTime = millis(); // Get the current time
  unsigned long elapsedTime;

  while (!radio.available()) { // Keep looping until data is available
    elapsedTime = millis() - startTime; // Calculate elapsed time
    if (elapsedTime >= 10) { // Adjust timeout duration (in milliseconds) as needed
      Serial.println("Timeout: No new node request received");
      return; // Exit the function if timeout occurs
    }
  }

  // If data is available, read the newNodeRequest
  radio.read(&newNodeRequest, sizeof(newNodeRequest));
  if (newNodeRequest) {
    Serial.println("New Node Request received from the receiver");
    // Process the new node request here
  }
}

//We need to use this to pass data to onboard arduino, 
//Make function for send array from Python -> Arduino (external) -> Arduino (onboard)
// Arduino (onboard) : should be able to translate the joystick controls to actual controls for robot enum commands
void process_python_data(){
  
  if(myTransfer.available())
  {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;

    recSize = myTransfer.rxObj(recieveStruct, recSize);
    sendStruct.x =recieveStruct.x;
    sendStruct.y = recieveStruct.y;

  }
  delay(500);

  uint16_t sendSize = 0;

  ///////////////////////////////////////// Stuff buffer with struct
  sendSize = myTransfer.txObj(sendStruct, sendSize);

  ///////////////////////////////////////// Send buffer
  myTransfer.sendData(sendSize);
  delay(500);    
  }



void process_python_data_list(){
  uint32_t arr[4];
  uint32_t new_arr[4]; 
  if(myTransfer.available())
  {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(arr, recSize);

    for (int i =0; i < sizeof(arr); i++){
      new_arr[i] = arr[i] + 1;
    }

  }
  delay(500);

  // uint32_t sendSize = 0;
  // uint32_t send_arr[] = {1, 3}; 
  // ///////////////////////////////////////// Stuff buffer with struct
  // sendSize = myTransfer.txObj(new_arr, sendSize);

  // ///////////////////////////////////////// Send buffer
  // myTransfer.sendData(sendSize);
  delay(500);    
  }



// [[1,3], 
// [1, 4], 
// [17, 83]]
