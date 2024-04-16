#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN pins

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    int receivedData[5];
    radio.read(&receivedData, sizeof(receivedData));
    
    Serial.println("Data received:");
    for (int i = 0; i < 5; i++) {
      Serial.print(receivedData[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
