#include <Arduino.h>

#define RXD2 16
#define TXD2 17  // not connected to anything, but required by HardwareSerial

HardwareSerial gpsSerial(2);

void setup() {
    Serial.begin(9600);       // USB to your PC
    gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("ESP32 GPS bridge ready");
}

void loop() {
    while (gpsSerial.available()) {
        Serial.write(gpsSerial.read());
    }
}