// ============================================================
// PING - ESP1 (Initiator)
// Sends first, then waits for reply, then sends again
// ============================================================
// ============================================================
// PING - ESP1 (Initiator)
// ============================================================
// #include <Arduino.h>

// #define RXD2  16
// #define TXD2  17

// void setup() {
//     Serial.begin(9600);
//     Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
//     delay(1000);
//     Serial.println("=== PING (ESP1) READY ===");
// }

// void loop() {
//     // --- SEND ---
//     Serial2.println("PING");
//     Serial.println(">> Sent: PING");

//     // --- WAIT FOR REPLY ---
//     Serial.println("   Waiting for PONG...");
//     unsigned long timeout = millis();
//     String received = "";

//     while (millis() - timeout < 5000) {
//         if (Serial2.available()) {
//             char c = Serial2.read();
//             if (c == '\n') {
//                 received.trim();
//                 if (received.length() > 0) {
//                     Serial.println("<< Received: " + received);
//                     received = "";
//                     break;
//                 }
//             } else {
//                 received += c;
//             }
//         }
//     }

//     if (millis() - timeout >= 5000) {
//         Serial.println("!! Timeout - no reply");
//     }

//     delay(1000);
// }

#include <Arduino.h>

#define RXD2 16
#define TXD2 17

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(1000);
    Serial.println("Receiver ready");
}

void loop() {
    if (Serial2.available()) {
        Serial.write(Serial2.read());
    }
}