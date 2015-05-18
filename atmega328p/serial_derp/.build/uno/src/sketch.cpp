#include <Arduino.h>

void setup();
void loop();
void receivedWord(uint32_t w);
#line 1 "src/sketch.ino"
/*
 * Veryify Connection
 *   Verify the connection between the atmega328p & the raspberry pi via uart handshake.
 *
 * Drew Troxell
 */ 

#define BUFFER_SIZE 4

uint8_t buf[BUFFER_SIZE];
uint16_t siz = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(D0, INPUT);

    while (Serial.available() <= 0) {
        Serial.println("HEYHEYHEY");
        delay(300);
    }
}


void loop() {
    uint16_t nBA = Serial.available();

    while (nBA-- > 0) {
        buf[siz++] = Serial.read();
        if (size == BUFFER_SIZE) {
            receivedWord(*(uint32_t *)buf);
            size = 0;
        }
    }

    delay(100);
}

void receivedWord(uint32_t w) {
    // echo
    Serial.write(&w, BUFFER_SIZE);
}
