#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <esp_now.h>
#include <WiFi.h>

#define BNO055_ADDR 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR);
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_angles {
  float angleY;
  float angleZ;
  bool  goFlag;
} struct_angles;
struct_angles angleData;

const int btnPin = 2;  // D2
bool goFlag = true;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(115200);
  pinMode(btnPin, INPUT_PULLUP);

  Wire.begin();
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected");
    while (1) delay(500);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 OK");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    while (1) delay(500);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: add_peer failed");
    while (1) delay(500);
  }

  Serial.println("ESP-NOW TX Ready");
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float y = euler.y();
  float z = euler.z();

  // Auto-stop if Y between 80–100 degrees
  if (goFlag && y >= 80 && y <= 100) {
    goFlag = false;
    Serial.println("Y angle in range 80–100 → Transmission stopped");
  }

  // Toggle goFlag with button only when it's OFF
  int reading = digitalRead(btnPin);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH && !goFlag) {
      goFlag = true;
      Serial.println("Button pressed → Transmission resumed");
    }
  }
  lastButtonState = reading;

  angleData.angleY = y;
  angleData.angleZ = z;
  angleData.goFlag = goFlag;

  // Send only when goFlag is true
  if (goFlag) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&angleData, sizeof(angleData));
    if (result == ESP_OK) {
      Serial.printf("Sent → Y: %.2f°, Z: %.2f°, go: %d\n", y, z, goFlag);
    } else {
      Serial.println("ERROR: Send failed");
    }
  }

  delay(100);
}