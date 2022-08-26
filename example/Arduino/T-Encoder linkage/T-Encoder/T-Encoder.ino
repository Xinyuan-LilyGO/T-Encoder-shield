#include <Arduino.h>
#include "HardwareSerial.h"

#define SHIELD_RX_PIN 4
#define SHIELD_TX_PIN 15

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, SHIELD_TX_PIN, SHIELD_RX_PIN);
}

void loop()
{

    while (Serial.available())
        Serial1.write(Serial.read());

    while (Serial1.available())
        Serial.write(Serial1.read());
}
