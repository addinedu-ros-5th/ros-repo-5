#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 5      // SDA(SS) - GPIO5
#define SCK_PIN 18    // SCK - GPIO18
#define MOSI_PIN 23   // MOSI - GPIO23
#define MISO_PIN 19   // MISO - GPIO19

// GND - GND 
#define RST_PIN 21 // RST - GPIO21
// VCC - 3V3
MFRC522 mfrc522(SS_PIN, RST_PIN);  

void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  mfrc522.PCD_Init();
  
  #
  byte v = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("MFRC522 Software Version: 0x");
  Serial.println(v, HEX);
  if (v == 0x91 || v == 0x92)
    Serial.println("MFRC522 detected.");
  else
    Serial.println("MFRC522 not detected. Check wiring.");
}

void loop() {
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(0x55);  // 임의의 데이터 전송
  digitalWrite(SS_PIN, HIGH);
  Serial.println("SPI transfer completed");
  delay(1000);

  Serial.println("Scanning...");
  
  if (!mfrc522.PICC_IsNewCardPresent()) {
    Serial.println("No new card");
    delay(500);
    return;
  }
  
  if (!mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Failed to read card");
    delay(500);
    return;
  }
  
  Serial.println("Card detected!");
  // 카드 정보 출력
  Serial.print("Card UID:");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println();
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  delay(1000);
}