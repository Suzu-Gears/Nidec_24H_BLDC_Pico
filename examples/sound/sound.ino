#include <Nidec_24H_BLDC_Pico.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel WS2812Bの指定
#define NUM_LEDS 3  // NeoPixelの数
#define LED_PIN 15  // NeoPixel信号接続端子
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// encoder_a, encoder_b, direction, pwm_pin, brake, reduction_ratio)
// Motor 1: Sound Source (制御対象)
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
// Motor 2: Amplitude Control (エンコーダ1: 振幅)
NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };
// Motor 3: Frequency Control (エンコーダ2: 周波数)
NidecBLDC motor3{ 16, 17, 18, 19, 20, 20.0f };

void setup() {
  strip.begin();                                  // NeoPixel初期化
  strip.show();                                   // NeoPixelを全て消灯
  strip.setBrightness(255);                       // 明るさ倍率設定 (RGB設定値 * 明るさ倍率、最大255)
  strip.setPixelColor(0, strip.Color(0, 32, 0));  // LED発光色指定
  strip.setPixelColor(1, strip.Color(0, 0, 32));
  strip.setPixelColor(2, strip.Color(32, 0, 0));
  strip.show();  // 点灯実行

  Serial.begin(115200);

  delay(1000);  // Wait for motors to stabilize

  if (!motor1.begin()) {
    Serial.println("Motor 1 Error");
    while (1);
  }
  if (!motor2.begin()) {
    Serial.println("Motor 2 Error");
    while (1);
  }
  if (!motor3.begin()) {
    Serial.println("Motor 3 Error");
    while (1);
  }

  motor1.setZeroPoint();
  motor2.setZeroPoint();
  motor3.setZeroPoint();

  motor1.setOutput(0);
}

void loop() {
  motor1.update();
  motor2.update();
  motor3.update();

  // --- Amplitude Control (Motor 2) ---
  // getRawPosition()を使用してエンコーダの生値を取得
  long rawPos2 = motor2.getRawPosition();
  // 適当なスケーリングで0~32767の範囲にする
  int32_t amplitude = abs(rawPos2 * 200);
  amplitude = constrain(amplitude, 0, 32767);

  // --- Frequency Control (Motor 3) ---
  long rawPos3 = motor3.getRawPosition();
  // 最低10Hz, 最大2000Hz程度に設定
  float frequency = abs(rawPos3 ) + 10.0f;
  frequency = constrain(frequency, 10.0f, 2000.0f);

  // --- Sound Generation (Motor 1) ---
  static unsigned long lastToggleMicros = 0;
  static bool outputState = false;
  unsigned long currentMicros = micros();
  unsigned long halfPeriod = (unsigned long)(1000000.0f / frequency / 2.0f);

  if (currentMicros - lastToggleMicros >= halfPeriod) {
    outputState = !outputState;
    lastToggleMicros = currentMicros;

    if (amplitude > 0) {
      // 正負を入れ替える
      motor1.setOutput(outputState ? amplitude : -amplitude);
    } else {
      motor1.setOutput(0);
    }
  }

  // --- Serial Output (1Hz) ---
  static unsigned long lastPrintMillis = 0;
  if (millis() - lastPrintMillis >= 1000) {
    Serial.print("Amp: ");
    Serial.print(amplitude);
    Serial.print(" (Raw2: ");
    Serial.print(rawPos2);
    Serial.print("), Freq: ");
    Serial.print(frequency);
    Serial.print(" Hz (Raw3: ");
    Serial.print(rawPos3);
    Serial.println(")");
    lastPrintMillis = millis();
  }
}
