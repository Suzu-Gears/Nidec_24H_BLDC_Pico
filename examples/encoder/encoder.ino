#include <Nidec_24H_BLDC_Pico.h>
#include <Adafruit_NeoPixel.h>
#include "pid_controller.hpp"

#define NUM_LEDS 1  // NeoPixelの数
#define LED_PIN 16  // NeoPixel信号接続端子
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

PidController motor1_pid;
float motor1_rad = 0.0f;

// encoder_a, encoder_b, direction, pwm_pin, brake, reduction_ratio)
NidecBLDC motor1{ 28, 27, 26, 15, 14, 1.0f };
// NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
// NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };
// NidecBLDC motor3{ 16, 17, 18, 19, 20, 20.0f };
// NidecBLDC motor4{ 21, 22, 26, 27, 28, 40.0f };

void setup() {
  strip.begin();                                  // NeoPixel初期化
  strip.show();                                   // NeoPixelを全て消灯
  strip.setBrightness(255);                       // 明るさ倍率設定 (RGB設定値 * 明るさ倍率、最大255)
  strip.setPixelColor(0, strip.Color(0, 32, 0));  // LED発光色指定（num,strip.Color（R,G,B））※RGB:0〜255
  strip.show();                                   // 点灯実行（NeoPixel制御パルス出力）

  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for Serial to initialize (max 3 seconds)

  delay(1000);  // Wait for motors to stabilize

  if (!motor1.begin()) {
    Serial.println("Motor 1 Encoder Error");
    while (1);
  }

  motor1.setZeroPoint();
  motor1_pid.init({ 0.01f, 4000.0f, 0.0f, 25.0f, 0.0f, 32767.0f });
}
void loop() {
  static unsigned long startTime = millis();
  static unsigned long lastToggleTime = 0;
  static bool toggle = false;  // State to toggle between ±180 degrees

  unsigned long currentTime = millis();
  if (currentTime - lastToggleTime >= 3000) {
    toggle = !toggle;
    float targetPosition = toggle ? 0.0f : 0.0f;             // ±180 degrees
    float targetPositionRad = targetPosition * PI / 180.0f;  // Convert degrees to radians
    motor1_rad = targetPositionRad;

    lastToggleTime = currentTime;
  }

  float motor1_output = motor1_pid.solve(motor1_rad - motor1.getRad());
  motor1.setOutput(motor1_output);

  motor1.update();

  Serial.print("[Motor Feedback] M1: ");
  Serial.print("Target Rad: ");
  Serial.print(motor1_rad, 4);
  Serial.print(", Current Rad: ");
  Serial.print(motor1.getRad(), 4);
  Serial.print(", Raw Position: ");
  Serial.print(motor1.getRawPosition());
  Serial.print(", Output: ");
  Serial.println(motor1_output, 4);

  delay(10);  // Delay for stability
}
