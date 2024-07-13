#include <SPI.h>
#include <MFRC522.h>
        
#define SS_PIN 5      // SDA(SS) - GPIO5
#define SCK_PIN 18    // SCK - GPIO18
#define MOSI_PIN 23   // MOSI - GPIO23
#define MISO_PIN 19   // MISO - GPIO19

// GND - GND 
#define RST_PIN 21 // RST - GPIO21
// VCC - 3V3

MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 RFID module

#define BUILTIN_LED_PIN 2           // 
#define GREEN_LED_PIN 12


void setup() {
  Serial.begin(115200);
  while (!Serial);  
  Serial.println("Serial communication started");

  SPI.begin(18, 19, 23, 5);  // SCK, MISO, MOSI, SS
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  mfrc522.PCD_Init();
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  SPI.setFrequency(4000000); // SPI Clock : 4MHz
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  
  pinMode(BUILTIN_LED_PIN, OUTPUT); 
  pinMode(GREEN_LED_PIN, OUTPUT); 

  // MFRC522 test
  // byte v = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  // Serial.print("MFRC522 Software Version: 0x");
  // Serial.println(v, HEX);
  // if (v == 0x91 || v == 0x92)
  //   Serial.println("MFRC522 detected.");
  // else
  //   Serial.println("MFRC522 not detected. Check wiring.");
    
  // Serial.println("Ready to scan RFID tags...");
}

void loop() {
  // RFID scan
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Card detected!");
    // card info print
    Serial.print("Card UID:");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();
    
    // LED on/off
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    Serial.println("LED turned ON");
    delay(1000);  
    digitalWrite(BUILTIN_LED_PIN, LOW);
    Serial.println("LED turned OFF");
    Serial.println("Scaned!");
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  } 
  else {
    Serial.println("Scanning...");
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(1000);  
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  

  delay(500);  
}