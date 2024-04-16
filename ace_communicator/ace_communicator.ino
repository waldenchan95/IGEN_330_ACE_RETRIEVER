#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};


#include "SerialTransfer.h"

// struct STRUCT{
//     int node1X; // Change this value to whatever you want to send (0-255) => positive values 0-128 negative values 129-256
//     int node1Y;
//     int node2X;
//     int node2Y;
//     int node3X;
//     int node3Y;
//     int node4X;
//     int node4Y;
//     int node5X;
//     int node5Y;
//     int node6X;
//     int node6Y;
//     int node7X;
//     int node7Y;
//     int node8X;
//     int node8Y;
// } node_coords;


SerialTransfer myTransfer;

void setup() {
  Serial.begin(115200);
  radio.begin();
  // myTransfer.begin(Serial);
  radio.openWritingPipe(thisSlaveAddress);
  radio.setPALevel(RF24_PA_MAX); // Set power level
  radio.stopListening(); // Set as transmitter
}

int receivedData[16];

void loop() {
    if (Serial.available() >= sizeof(int) * 16) { // Check if all bytes have been received
    // int receivedData[16]; // recievedDataay to store received integers
    Serial.readBytes((char *)receivedData, sizeof(int) * 16); // Read bytes into the recievedDataay
    for (int i = 0; i < 5; i++) {
      Serial.print("Received Data[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(receivedData[i]);
    }
    
    // Send acknowledgment when all the data processed
    Serial.println("ACK");
    // Process received data
    byte infoPacket = 69;
    byte node1X = receivedData[0]; // Change this value to whatever you want to send (0-255) => positive values 0-128 negative values 129-256
    byte node1Y = receivedData[1];
    byte node2X = receivedData[2];
    byte node2Y = receivedData[3];
    byte node3X = receivedData[4];
    byte node3Y = receivedData[5];
    byte node4X = receivedData[6];
    byte node4Y = receivedData[7];
    byte node5X = receivedData[8]; 
    byte node5Y = receivedData[9];
    byte node6X = receivedData[10]; 
    byte node6Y = receivedData[11];
    byte node7X = receivedData[12]; 
    byte node7Y = receivedData[13];
    byte node8X = receivedData[14]; 
    byte node8Y = receivedData[15];
    // byte dataToSend[17] = {node1X, node1Y, node2X, node2Y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, infoPacket};
    byte dataToSend[17] = {node1X, node1Y, node2X, node2Y, node3X, node3Y, node4X, node4Y, node5X, node5Y, node6X, node6Y, node7X, node7Y, node8X, node8Y, infoPacket};
    // byte dataToSend[17] = {node1X, node5X, node1Y, node5Y, node2X, node6X, node2Y, node6Y, node3X, node7X, node3Y, node7Y, node4X, node8X, node4Y, node8Y, infoPacket};
    
    // Send the data
    radio.write(&dataToSend, sizeof(dataToSend));
    
    // Serial.print("Data sent: ");
    
    // Serial.print(dataToSend[0]);
    // Serial.print("-");

    // Serial.print(dataToSend[1]);
    // Serial.print("-");

    // Serial.print(dataToSend[2]);
    // Serial.print("-");

    // Serial.print(dataToSend[3]);
    // Serial.print("-");

    // Serial.print(dataToSend[4]);
    // Serial.print("-");

    // Serial.print(dataToSend[5]);
    // Serial.print("-");

    // Serial.print(dataToSend[6]);
    // Serial.print("-");

    // Serial.print(dataToSend[7]);
    // Serial.print("-");

    // Serial.print(dataToSend[8]);
    // Serial.print("-");

    // Serial.print(dataToSend[9]);
    // Serial.print("-");

    // Serial.print(dataToSend[10]);
    // Serial.print("-");

    // Serial.print(dataToSend[11]);
    // Serial.print("-");

    // Serial.print(dataToSend[12]);
    // Serial.print("-");

    // Serial.print(dataToSend[13]);
    // Serial.print("-");

    // Serial.print(dataToSend[14]);
    // Serial.print("-");

    // Serial.print(dataToSend[15]);
    // Serial.print("  ");

    // Serial.print(" InfoPacket: ");
    // Serial.println(dataToSend[16]);
  }
}

    
  
  

