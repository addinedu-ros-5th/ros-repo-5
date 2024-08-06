#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         9          
#define SS_PIN          10        
#define MOSI_PIN        11
#define MISO_PIN        12
#define SCK_PIN         13


MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

// Define a struct to hold UID information
struct UIDInfo {
    byte uid[3];
    byte uidSize;
};

// Array to store the detected UIDs
UIDInfo detectedUIDs[10];
int detectedUIDCount = 0;
const int MAX_UIDS = 10;

void setup() {
    Serial.begin(9600);     // Initialize serial communications with the PC
    while (!Serial);        // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
    SPI.begin();            // Init SPI bus
    mfrc522.PCD_Init();     // Init MFRC522
    delay(4);               // Optional delay. Some boards do need more time after init to be ready, see Readme
    mfrc522.PCD_DumpVersionToSerial(); // Show details of PCD - MFRC522 Card Reader details
    Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void loop() {
    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    if (!mfrc522.PICC_IsNewCardPresent()) {
        return;
    }

    // Select one of the cards
    if (!mfrc522.PICC_ReadCardSerial()) {
        return;
    }

    // Store the detected UID
    storeUID(mfrc522.uid.uidByte, mfrc522.uid.size);

}

void storeUID(byte *uid, byte uidSize) {
    if (detectedUIDCount >= MAX_UIDS) {
        Serial.println(F("UID storage full!"));
        return;
    }

    // Store the UID in the array
    for (byte i = 0; i < uidSize; i++) {
        detectedUIDs[detectedUIDCount].uid[i] = uid[i];
    }
    detectedUIDs[detectedUIDCount].uidSize = uidSize;
    detectedUIDCount++;

    // Print the stored UID
    Serial.print(F("Stored UID: "));
    for (byte i = 0; i < uidSize; i++) {
        Serial.print(uid[i] < 0x10 ? " 0" : " ");
        Serial.print(uid[i], HEX);
    }
    Serial.println();
}
