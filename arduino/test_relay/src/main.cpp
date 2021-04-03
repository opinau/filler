#include <Arduino.h>

int relayArray[] = {2, 3, 4, 5, 6, 7, 8, 9};

int delayTime = 100;

void setup() {
    for (int i = 0; i < 8; i++) {
        pinMode(relayArray[i], OUTPUT);
    }
}

void loop() {
    for (int i = 0; i <= 7; i++) {
        digitalWrite(relayArray[i], LOW);
        delay(delayTime);
    }

    for (int i = 7; i >= 0; i--) {
        digitalWrite(relayArray[i], LOW);
        delay(delayTime);
    }
}
