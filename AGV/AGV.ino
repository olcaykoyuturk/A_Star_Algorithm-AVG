// LIBRARY
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

// CONNECT WIFI
const char* ssid = "ESP32-AP";
const char* password = "12345678";

WebSocketsClient webSocket;

// ID
const char* deviceID = "avg1";

// 74HC4051 MULTIPLEXER
#define S0 21
#define S1 22
#define S2 23
#define Z_PIN 36

// MOSFET GATE PIN
#define MOSFET_PIN 19

// MOTOR-A (left)
const int PWMA = 14;
const int AIN1 = 12;
const int AIN2 = 13;

// MOTOR-B (right)
const int PWMB = 25;
const int BIN1 = 26;
const int BIN2 = 27;

const int STBY = 33;

// MOTOR-PWM SETTINGS
const int freq = 1000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

// SERVO-MOTOR
const int servoYukariAsagi = 17;
const int servoSagSol = 16;

// SERVO-PWM SETTINGS
const int Servo_pwmChannel1 = 2;
const int Servo_pwmChannel2 = 3;
const int Servo_freq = 50;
const int Servo_resolution = 12;
const int Servo_minUs = 500;
const int Servo_maxUs = 2500;

// PID
float Kp = 0.04;
float Kd = 0.08;
int oncekiHata = 0;
unsigned int speed = 120;

// MAP
int8_t x = 0;
int8_t y = 0;
int8_t target_x = 0;
int8_t target_y = 0;

bool hedefeUlasildi = true;

enum Yon { KUZEY,
           GUNEY,
           DOGU,
           BATI };

Yon yon = KUZEY;

// LINE POZISYON
unsigned int sonPozisyon = 3500;

// TIME
unsigned long sonCollisionZamani = 0;

// LINE PINS
uint16_t sensorDegerleri[8];

void setup() {
  Serial.begin(115200);

  // LED
  pinMode(2, OUTPUT);

  // MULTIPEXER
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  // MOTOR DRIVER
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(PWMA, pwmChannelA);

  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(PWMB, pwmChannelB);

  digitalWrite(STBY, HIGH);

  // SERVO
  ledcSetup(Servo_pwmChannel1, Servo_freq, Servo_resolution);
  ledcAttachPin(servoSagSol, Servo_pwmChannel1);

  ledcSetup(Servo_pwmChannel2, Servo_freq, Servo_resolution);
  ledcAttachPin(servoYukariAsagi, Servo_pwmChannel2);

  setServoAngle(Servo_pwmChannel1, 90);
  setServoAngle(Servo_pwmChannel2, 90);
  delay(500);
  stopServo(Servo_pwmChannel1);
  stopServo(Servo_pwmChannel2);

  // MOSFET
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  connectToWiFi();
  delay(50);
  setupWebSocket();

  // WAIT
  for (int i = 0; i < 10; i++) {
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
  }
}

void loop() {
  webSocket.loop();

  if (hedefeUlasildi) {
    motorAll_dur();
    return;
  }

  // CIZGI SENSORU
  sensorleriOku();
  unsigned int cizgiPozisyonu = readLineBlack(sensorDegerleri);

  // HAREKET
  collision();
  PID(cizgiPozisyonu);
}