#include <esp_now.h>
#include <WiFi.h>

// —— Data struct (must match your transmitter) ——
typedef struct struct_angles {
  float angleY;
  float angleZ;
  bool  goFlag;
} struct_angles;

// —— Motor driver pin definitions ——
const int IN1 = 4;   // side A dir 1 (Out1)
const int IN2 = 5;  // side A dir 2 (Out2)
const int ENA = 3;   // side A enable  (motors 1&2)
const int IN3 = 7;   // side B dir 1 (Out3)
const int IN4 = 6;   // side B dir 2 (Out4)
const int ENB = 8;   // side B enable  (motors 3&4)

// —— PWM config ——
const int freq = 1000;        // 1 kHz PWM frequency
const int resolution = 8;     // 8-bit resolution (0–255)
const int pwmChannelA = 0;
const int pwmChannelB = 1;
int motorSpeed = 200;          // 0 (stop) to 255 (full speed)

// —— Drive routines ——
void driveStop() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}

void driveForward() {
  // Side A forward (LOW = forward)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Side B forward (inverted wiring)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(pwmChannelA, motorSpeed);
  ledcWrite(pwmChannelB, motorSpeed);
}

void driveBackward() {
  // Side A backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Side B backward (inverted wiring)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(pwmChannelA, motorSpeed);
  ledcWrite(pwmChannelB, motorSpeed);
}

void turnLeft() {
  // Pivot left: left side backward (A), right side forward (B)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(pwmChannelA, motorSpeed);
  ledcWrite(pwmChannelB, motorSpeed);
}

void turnRight() {
  // Pivot right: left side forward (A), right side backward (B)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(pwmChannelA, motorSpeed);
  ledcWrite(pwmChannelB, motorSpeed);
}

// —— Pick motion based on angles + goFlag ——
void decideDirection(float y, float z, bool goFlag) {
  if (!goFlag) {
    driveStop();
    return;
  }

  if (y >= 30 && y <= 60) {
    Serial.println("Command: FORWARD");
    driveForward();
  }
  else if (y <= -60 && y >= -120) {
    Serial.println("Command: BACKWARD");
    driveBackward();
  }
  else if (z <= -60 && z >= -120) {
    Serial.println("Command: LEFT");
    turnLeft();
  }
  else if (z >= 60 && z <= 120) {
    Serial.println("Command: RIGHT");
    turnRight();
  }
  else {
    Serial.println("Command: STOP (Neutral Tilt)");
    driveStop();
  }
}

// —— ESP-NOW receive callback ——
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  struct_angles d;
  memcpy(&d, data, sizeof(d));
  Serial.printf("Y: %.2f  Z: %.2f  goFlag: %d\n", d.angleY, d.angleZ, d.goFlag);
  decideDirection(d.angleY, d.angleZ, d.goFlag);
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // PWM Setup
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(ENB, pwmChannelB);

  // Wi‑Fi / ESP‑NOW init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP‑NOW init failed");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Receiver ready");
}

void loop() {
  // all action in onDataRecv()
  delay(10);
}