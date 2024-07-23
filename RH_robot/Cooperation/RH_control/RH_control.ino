#include <SPI.h>
#include <MFRC522.h>
#include "/home/jinsa/git_wp/ros-repo-5/RH_lot/ESP32_RFIDs_check/config.h"

#define BTN_PIN 4
#define LED_PIN 7

#define RST_PIN 9          
#define SS_PIN 10          

MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 instance
MFRC522::MIFARE_Key key;

bool rfidReady = false;

void setup() {
  Serial.begin(9600);   
  SPI.begin();          
  mfrc522.PCD_Init();  
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);  
}

void loop() {
  // 1. 버튼이 눌리면 LED를 켜고 RFID 인식 준비
  if (digitalRead(BTN_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
    rfidReady = true;
    Serial.println("RFID recognition ready. Give me your card.");
    delay(200);  // 디바운싱
  }

  // RFID 인식이 준비되었을 때만 카드 검사
  if (rfidReady && mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    // 2. RFID 카드 정보 읽기
    byte readCardUID[4];
    for (byte i = 0; i < 4; i++) {
      readCardUID[i] = mfrc522.uid.uidByte[i];
    }

    // 3. config.h에서 카드 정보 확인
    bool cardFound = false;
    for (int i = 0; i < sizeof(targetUIDs) / sizeof(targetUIDs[0]); i++) {
      if (memcmp(readCardUID, targetUIDs[i].uid, 4) == 0) {
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
      Serial.println("등록되지 않은 카드입니다.");
    }

    digitalWrite(LED_PIN, LOW);
    rfidReady = false;
    
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}