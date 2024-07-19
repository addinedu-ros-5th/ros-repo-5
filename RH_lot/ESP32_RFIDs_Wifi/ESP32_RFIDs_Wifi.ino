#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <MFRC522.h>

// RFID 1
#define SS_PIN 5      // SDA(SS) - GPIO5
#define SCK_PIN 18    // SCK - GPIO18
#define MOSI_PIN 23   // MOSI - GPIO23
#define MISO_PIN 19   // MISO - GPIO19
#define RST_PIN 21    // RST - GPIO21

// RFID 2
#define SS_PIN2 26    // SDA(SS) - GPIO26
//#define SCK_PIN 18    // SCK - GPIO18
//#define MOSI_PIN 23   // MOSI - GPIO23
//#define MISO_PIN 19   // MISO - GPIO19
#define RST_PIN2 13   // RST - GPIO21

#define BUILTIN_LED_PIN 2 // BUILTIN_LED_PIN

byte targetUID[4] = {0xC3, 0x49, 0x6F, 0x96};
MFRC522 mfrc522(SS_PIN, RST_PIN);   // MFRC522 RFID module
MFRC522 mfrc522_2(SS_PIN2, RST_PIN2);   // MFRC522 RFID module


const char *ssid = "addinedu_class_1(2.4G)";
const char *password = "addinedu1";

AsyncWebServer server(80);
bool ledState = false;

// Web page
const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<body>
<center>
<h1>Hello, ESP32 Web Server!</h1>
<div>BUILTIN_LED PIN 2 : 
<input type="checkbox" id="ledCheckbox" onchange="toggleCheckBox(this)" />
</div>
<div id="ledStatus">LED Status: OFF</div>
<script>
function toggleCheckBox(element){
  var req = new XMLHttpRequest();
  if (element.checked){
    req.open("GET", "/on", true);
  } 
  else{
    req.open("GET", "/off", true);
  }
  req.send();
}

function updateLedState() {
  var req = new XMLHttpRequest();
  req.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var isOn = this.responseText === "1";
      document.getElementById("ledCheckbox").checked = isOn;
      document.getElementById("ledStatus").innerHTML = "LED Status: " + (isOn ? "ON" : "OFF");
    }
  };
  req.open("GET", "/ledstate", true);
  req.send();
}

setInterval(updateLedState, 1000);
</script>
</center>
</body>
</html>
)rawliteral";

String processor(const String& var){
  Serial.println(var);
  return var;
}

void checkRFID(MFRC522 &rfid); 

void setup() {
  

  Serial.begin(115200);
  Serial.println("ESP32 Web Server Start");
  Serial.println(ssid);

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


  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());



  server.on("/", HTTP_GET, [] (AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", html, processor);
  });
  server.on("/on", HTTP_GET, [] (AsyncWebServerRequest *req) {
    Serial.println("on1");
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    req->send_P(200, "text/html", html, processor);
  });
  server.on("/off", HTTP_GET, [] (AsyncWebServerRequest *req) {
    Serial.println("off1");
    digitalWrite(BUILTIN_LED_PIN, LOW);
    req->send_P(200, "text/html", html, processor);
  });
  server.on("/ledstate", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "text/plain", ledState ? "1" : "0");
  });

  server.begin();

  Serial.println("HTTP Server Started!");
  delay(100);
}

void loop() {
  checkRFID(mfrc522);
  checkRFID(mfrc522_2);
  delay(500);
  
}

void checkRFID(MFRC522 &rfid) {
  // RFID scan
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.println("Card detected!");
    // Output card information
    Serial.print("Card UID:");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();
    // Compare the scanned UID with the target UID
    bool isMatch = true;
    for (byte i = 0; i < 4; i++) {
      if (rfid.uid.uidByte[i] != targetUID[i]) {
        isMatch = false;
        break;
      }
    }
    if (isMatch) {
      digitalWrite(BUILTIN_LED_PIN, HIGH);
      ledState = true;  // 
      Serial.println("Matching card detected. LED turned ON.");
      delay(3000);
      digitalWrite(BUILTIN_LED_PIN, LOW);
      ledState = false;  // 
      Serial.println("LED turned OFF after 3 seconds.");
    } else {
      digitalWrite(BUILTIN_LED_PIN, LOW);
      ledState = false;  // 
      Serial.println("Non-matching card detected. LED turned OFF.");
    }
    Serial.println("Scanned!");
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
}