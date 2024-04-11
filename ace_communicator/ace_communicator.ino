#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Define the pins for the nRF24L01 module
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

const uint64_t pipe = 0xE8E8F0F0E1LL;  // Define the communication pipe address

void setup() {
    Serial.begin(9600);
    radio.begin();
    //radio.setDataRate(RF24_250KBPS);
    radio.openWritingPipe(pipe);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        int data_size = Serial.read(); // Read the array size
        int data_array[data_size];

        for (int i = 0; i < data_size; i++) {
            data_array[i] = Serial.read(); // Read each element of the array
        }
        if (data_size > 0){
          digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
          delay(1000);                      // wait for a second
          digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
          delay(1000); 
          digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
          delay(1000);                      // wait for a second
          digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
          delay(1000); 
        }
        
        // Send the array via nRF24L01
        radio.write(&data_size, sizeof(data_size));
        delay(500);
        radio.write(data_array, sizeof(data_array));

        // Print the sent array to Serial Monitor for debugging
        for (int i = 0; i < data_size; i++) {
            Serial.print(data_array[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
}
