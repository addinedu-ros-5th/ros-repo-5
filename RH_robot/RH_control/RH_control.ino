#include <SPI.h>
#include <MFRC522.h>
#include "config.h"

#define SOLENOID 3   // Define digital port 3 for Solenoid
#define LED_PIN 7
#define RST_PIN 9          
#define SS_PIN 10          

MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 instance
MFRC522::MIFARE_Key key;

bool rfidReady = false;
bool solenoidState = false; // Track the state of the solenoid
Config config; // Create an instance of Config
int currentParkNum = -1; // Store the current park number

void setup() {
  Serial.begin(9600);   
  SPI.begin();          
  mfrc522.PCD_Init();  
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOLENOID, OUTPUT);

  digitalWrite(LED_PIN, LOW);  
  digitalWrite(SOLENOID, LOW); // Ensure solenoid is initially off
}

void loop() {
  // Check if there's any data available on the serial port
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "toggle_solenoid") {
      solenoidState = !solenoidState;
      digitalWrite(SOLENOID, solenoidState ? HIGH : LOW);
      Serial.println(solenoidState ? "STATUS:Solenoid ON" : "STATUS:Solenoid OFF");
    } else {
      currentParkNum = input.toInt();
      rfidReady = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("STATUS:Received index: " + String(currentParkNum));
      Serial.println("STATUS:RFID recognition ready. Give me your card.");
      sendStatus(); // Send initial status
    }
  }

  // Only check for the card if RFID recognition is ready
  if (rfidReady && mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    // Read RFID card information
    byte readCardUID[4];
    for (byte i = 0; i < 4; i++) {
      readCardUID[i] = mfrc522.uid.uidByte[i];
    }

    // Check card information from config.h
    bool cardFound = false;
    for (int i = 0; i < config.NUM_UIDS; i++) {
      if (memcmp(readCardUID, config.targetUIDs[i].uid, 4) == 0 && 
          config.targetUIDs[i].index == currentParkNum) {
        Serial.println("STATUS:Recognition successful!");
        Serial.print("UID:");
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
      Serial.println("STATUS:Invalid card or incorrect parking area.");
    }

    if (cardFound) {
      solenoidState = !solenoidState;
      digitalWrite(SOLENOID, solenoidState ? HIGH : LOW);
      Serial.println(solenoidState ? "STATUS:Solenoid ON" : "STATUS:Solenoid OFF");
    }

    digitalWrite(LED_PIN, LOW);
    rfidReady = false;
    
    sendStatus(); // Send status after processing RFID

    // Halt PICC and stop encryption on PCD
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}

void sendStatus() {
  String ledState = digitalRead(LED_PIN) == HIGH ? "true" : "false";
  String solenoidStateStr = solenoidState ? "true" : "false";
  String rfidReadyStr = rfidReady ? "true" : "false";
  
  Serial.println("STATUS:LEDState=" + ledState + ",solenoidState=" + solenoidStateStr + ",rfidReady=" + rfidReadyStr);
}
