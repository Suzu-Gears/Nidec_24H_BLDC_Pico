#include <Nidec_24H_BLDC_Pico.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include "pid_controller.hpp"

// NeoPixel WS2812Bの指定
#define NUM_LEDS 3  // NeoPixelの数
#define LED_PIN 15  // NeoPixel信号接続端子
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// encoder_a, encoder_b, direction, pwm_pin, brake, reduction_ratio)
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };
NidecBLDC motor3{ 16, 17, 18, 19, 20, 20.0f };
NidecBLDC motor4{ 21, 22, 26, 27, 28, 40.0f };

PidController motor1_pid;
PidController motor2_pid;
PidController motor3_pid;
PidController motor4_pid;

float motor1_rad = 0.0f;
float motor2_rad = 0.0f;
float motor3_rad = 0.0f;
float motor4_rad = 0.0f;

void setup() {
  strip.begin();                                  // NeoPixel初期化
  strip.show();                                   // NeoPixelを全て消灯
  strip.setBrightness(255);                       // 明るさ倍率設定 (RGB設定値 * 明るさ倍率、最大255)
  strip.setPixelColor(0, strip.Color(0, 32, 0));  // LED発光色指定（num,strip.Color（R,G,B））※RGB:0〜255
  strip.setPixelColor(1, strip.Color(0, 32, 0));  // LED発光色指定（num,strip.Color（R,G,B））※RGB:0〜255
  strip.setPixelColor(2, strip.Color(0, 32, 0));  // LED発光色指定（num,strip.Color（R,G,B））※RGB:0〜255
  strip.show();                                   // 点灯実行（NeoPixel制御パルス出力）

  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for Serial to initialize (max 3 seconds)

  delay(1000);  // Wait for motors to stabilize

  if (!motor1.begin()) {
    Serial.println("Motor 1 Encoder Error");
    while (1);
  }
  if (!motor2.begin()) {
    Serial.println("Motor 2 Encoder Error");
    while (1);
  }
  if (!motor3.begin()) {
    Serial.println("Motor 3 Encoder Error");
    while (1);
  }
  if (!motor4.begin()) {
    Serial.println("Motor 4 Encoder Error");
    while (1);
  }

  motor1_pid.init({ 0.01f, 2000.0f, 0.0f, 25.0f, 0.0f, 32767.0f });
  motor2_pid.init({ 0.01f, 2000.0f, 0.0f, 25.0f, 0.0f, 32767.0f });
  motor3_pid.init({ 0.01f, 8000.0f, 0.0f, 25.0f, 0.0f, 32767.0f });
  motor4_pid.init({ 0.01f, 12000.0f, 0.0f, 25.0f, 0.0f, 32767.0f });

  motor1.setZeroPoint();
  motor2.setZeroPoint();
  motor3.setZeroPoint();
  motor4.setZeroPoint();
}

void loop() {
  static unsigned long startTime = millis();
  static unsigned long lastToggleTime = 0;
  static bool toggle = false;  // State to toggle between ±180 degrees

  unsigned long currentTime = millis();
  if (currentTime - lastToggleTime >= 3000) {
    toggle = !toggle;
    float targetPosition = toggle ? 90.0f : -90.0f;          // ±180 degrees
    float targetPositionRad = targetPosition * PI / 180.0f;  // Convert degrees to radians

    motor1_rad = targetPositionRad;
    motor2_rad = targetPositionRad;
    motor3_rad = targetPositionRad;
    motor4_rad = targetPositionRad;

    lastToggleTime = currentTime;
  }

  float motor1_output = motor1_pid.solve(motor1_rad - motor1.getRad());
  motor1.setOutput(-motor1_output);
  float motor2_output = motor2_pid.solve(motor2_rad - motor2.getRad());
  motor2.setOutput(-motor2_output);
  float motor3_output = motor3_pid.solve(motor3_rad - motor3.getRad());
  motor3.setOutput(-motor3_output);
  float motor4_output = motor4_pid.solve(motor4_rad - motor4.getRad());
  motor4.setOutput(-motor4_output);

  // Update motors
  motor1.update();
  motor2.update();
  motor3.update();
  motor4.update();

  // Print detailed motor feedback
  Serial.print("[Motor Feedback] M1: ");
  Serial.print("Target Rad: ");
  Serial.print(motor1_rad, 4);
  Serial.print(", Current Rad: ");
  Serial.print(motor1.getRad(), 4);
  Serial.print(", Output: ");
  Serial.print(motor1_output, 4);

  Serial.print(" | M2: ");
  Serial.print("Target Rad: ");
  Serial.print(motor2_rad, 4);
  Serial.print(", Current Rad: ");
  Serial.print(motor2.getRad(), 4);
  Serial.print(", Output: ");
  Serial.print(motor2_output, 4);

  Serial.print(" | M3: ");
  Serial.print("Target Rad: ");
  Serial.print(motor3_rad, 4);
  Serial.print(", Current Rad: ");
  Serial.print(motor3.getRad(), 4);
  Serial.print(", Output: ");
  Serial.print(motor3_output, 4);

  Serial.print(" | M4: ");
  Serial.print("Target Rad: ");
  Serial.print(motor4_rad, 4);
  Serial.print(", Current Rad: ");
  Serial.print(motor4.getRad(), 4);
  Serial.print(", Output: ");
  Serial.println(motor4_output, 4);

  delay(10);  // Delay for stability
}
