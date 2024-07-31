#include <SPI.h>
#include <MFRC522.h>
#include "config.h"

#define SOLENOID 3   // Define digital port 3 for Solenoid
#define BTN_PIN 4
#define LED_PIN 7
#define RST_PIN 9          
#define SS_PIN 10          

MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 instance
MFRC522::MIFARE_Key key;

bool rfidReady = false;
bool solenoidState = false; // Track the state of the solenoid
Config config; // Create an instance of Config

void setup() {
  Serial.begin(9600);   
  SPI.begin();          
  mfrc522.PCD_Init();  
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOLENOID, OUTPUT);

  digitalWrite(LED_PIN, LOW);  
  digitalWrite(SOLENOID, LOW); // Ensure solenoid is initially off
}

void loop() {
  // 1. When the button is pressed, prepare for RFID recognition
  if (digitalRead(BTN_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
    rfidReady = true;
    Serial.println("RFID recognition ready. Give me your card.");
    
    delay(200);  // Debounce delay
  }

  // Only check for the card if RFID recognition is ready
  if (rfidReady && mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    // 2. Read RFID card information
    byte readCardUID[4];
    for (byte i = 0; i < 4; i++) {
      readCardUID[i] = mfrc522.uid.uidByte[i];
    }

    // 3. Check card information from config.h
    bool cardFound = false;
    for (int i = 0; i < config.NUM_UIDS; i++) {
      if (memcmp(readCardUID, config.targetUIDs[i].uid, 4) == 0) {
        Serial.println("Recognition successful!");
        Serial.print("UID: ");
        for (byte j = 0; j < 4; j++) {
          Serial.print(readCardUID[j] < 0x10 ? "0" : "");
          Serial.print(readCardUID[j], HEX);
        }
        Serial.println();
        cardFound = true;
        break;
      }
    }

    if (!cardFound) {
      Serial.println("해당 주차구역이 아닙니다.");
    }

    // Toggle the solenoid state
    if (cardFound) {
      solenoidState = !solenoidState;
      digitalWrite(SOLENOID, solenoidState ? HIGH : LOW);
      Serial.println(solenoidState ? "Solenoid ON" : "Solenoid OFF");
    }

    digitalWrite(LED_PIN, LOW);
    rfidReady = false;
    
    // Halt PICC and stop encryption on PCD
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}
