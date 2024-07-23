#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>
#include "config.h"

Config config;

// RFID 1
#define SS_PIN 5      // SDA(SS) - GPIO5
#define SCK_PIN 18    // SCK - GPIO18
#define MOSI_PIN 23   // MOSI - GPIO23
#define MISO_PIN 19   // MISO - GPIO19
#define RST_PIN 21    // RST - GPIO21
// RFID 2
#define SS_PIN2 26    // SDA(SS) - GPIO26
#define RST_PIN2 13   // RST - GPIO21
#define BUILTIN_LED_PIN 2 // BUILTIN_LED_PIN
MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 RFID module
MFRC522 mfrc522_2(SS_PIN2, RST_PIN2);   // MFRC522 RFID module
WiFiClient client;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial communication started");
  // WiFi 연결
  WiFi.begin(config.ssid, config.password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);  // SCK, MISO, MOSI, SS for the first reader
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN2);  // SCK, MISO, MOSI, SS for the second reader
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  pinMode(SS_PIN2, OUTPUT);
  digitalWrite(SS_PIN2, HIGH);
  mfrc522.PCD_Init();
  mfrc522_2.PCD_Init();
  SPI.setFrequency(4000000); // SPI Clock : 4MHz
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  Serial.println("Ready to scan RFID tags...");
}
void loop() {
  checkRFID(mfrc522);
  checkRFID(mfrc522_2);
  delay(500);
}
void checkRFID(MFRC522 &rfid) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        Serial.println("Card detected!");
        String uidString = "";
        for (byte i = 0; i < rfid.uid.size; i++) {
            uidString += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
            uidString += String(rfid.uid.uidByte[i], HEX);
        }
        Serial.println("Card UID: " + uidString);
        bool isMatch = false;
        int matchedIndex = -1;
        for (int i = 1; i < config.NUM_UIDS; i++) {  // Start from 1 to skip the empty UID at index 0
            bool currentMatch = true;
            for (byte j = 0; j < 4; j++) {
                if (rfid.uid.uidByte[j] != config.targetUIDs[i].uid[j]) {
                    currentMatch = false;
                    break;
                }
            }
            if (currentMatch) {
                isMatch = true;
                matchedIndex = i;
                break;
            }
        }
        if (isMatch) {
            digitalWrite(BUILTIN_LED_PIN, HIGH);
            Serial.println("Matching card detected. LED turned ON.");
            Serial.print("Matched UID Index: ");
            Serial.println(matchedIndex);
            sendToServer("MATCH:" + uidString + ",INDEX:" + String(matchedIndex));
            delay(3000);
            digitalWrite(BUILTIN_LED_PIN, LOW);
            Serial.println("LED turned OFF after 3 seconds.");
        } else {
            digitalWrite(BUILTIN_LED_PIN, LOW);
            Serial.println("Non-matching card detected. LED turned OFF.");
            sendToServer("NOT MATCH:" + uidString);
        }
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
    }
}
void sendToServer(String message) {
  if (client.connect(config.tcp_server, config.tcp_port)) {
    Serial.println("Connected to server");
    client.println(message);
    Serial.println("Message sent: " + message);
    client.stop();
    Serial.println("Disconnected from server");
  } else {
    Serial.println("Connection to server failed");
  }
}