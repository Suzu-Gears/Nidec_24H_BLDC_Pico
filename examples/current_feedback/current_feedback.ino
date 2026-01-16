#include <Nidec_24H_BLDC_Pico.h>
#include <Adafruit_NeoPixel.h>
#include "pid_controller.hpp"
#include "low_pass.h"

#include <Wire.h>

#define NUM_LEDS 1  // NeoPixelの数
#define LED_PIN 16  // NeoPixel信号接続端子
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

const int USER_BUTTON = 29;

int16_t BusVoltage, ShuntVoltage;

PidController motor1_pid;
LowPassFilter filter;

float targetCurrent = 50.0f;
float filteredCurrent = 0.0f;
float motor1_output = 0.0f;

// encoder_a, encoder_b, direction, pwm_pin, brake, reduction_ratio)
NidecBLDC motor1{ 28, 27, 26, 15, 14, 1.0f };

const float target_current_list[] = { 0.0f, 100.0f, -100.0f, 500.0f, -500.0f, 1000.0f, -1000.0f, 1200.0f, -1200.0f };
int target_list_index = 0;

void changeTargetCurrent() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    // インデックスを進める（リストの末尾を超えたら0に戻る）
    target_list_index = (target_list_index + 1) % (sizeof(target_current_list) / sizeof(target_current_list[0]));

    targetCurrent = target_current_list[target_list_index];
    motor1_pid.reset();
    last_interrupt_time = interrupt_time;
  }
}
void setup() {
  strip.begin();                                  // NeoPixel初期化
  strip.show();                                   // NeoPixelを全て消灯
  strip.setBrightness(255);                       // 明るさ倍率設定 (RGB設定値 * 明るさ倍率、最大255)
  strip.setPixelColor(0, strip.Color(0, 32, 0));  // LED発光色指定（num,strip.Color（R,G,B））※RGB:0〜255
  strip.show();                                   // 点灯実行（NeoPixel制御パルス出力）

  Wire.setSDA(4);
  Wire.setSCL(5);
  // Wire.setClock(400000); //400kHz
  Wire.setClock(2560000);         // 2.56MHz 2560kHz
  Wire.begin();                   // I2C0 begin
  int16_t config_value = 0x399F;  // INA219 デフォルト
  Wire.beginTransmission(0x40);   // INA219 デバイスアドレス
  Wire.write(0x00);               // configアドレス
  Wire.write((config_value >> 8) & 0xFF);
  Wire.write(config_value & 0xFF);
  Wire.endTransmission();

  delay(1000);  // Wait for motors to stabilize

  if (!motor1.begin()) {
    Serial.println("Motor 1 Encoder Error");
    while (1);
  }

  motor1.setZeroPoint();
  motor1_pid.init({ 0.01f, 100.0f, 1000.0f, 0.0f, 0.0f, 32767.0f, 30.0f });
  filter.init(0.1);

  pinMode(USER_BUTTON, INPUT);
  attachInterrupt(USER_BUTTON, changeTargetCurrent, FALLING);
}

void loop() {
  Wire.beginTransmission(0x40);  //デバイスアドレス
  Wire.write(0x02);              //BusVoltageレジスタ
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  while (Wire.available() < 2);
  uint16_t rawBusVoltage = Wire.read() << 8 | Wire.read();
  BusVoltage = (rawBusVoltage >> 3) * 4;

  //Get ShuntVoltage
  Wire.beginTransmission(0x40);  //デバイスアドレス
  Wire.write(0x01);              //ShuntVoltageレジスタ
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  while (Wire.available() < 2);
  ShuntVoltage = Wire.read() << 8 | Wire.read();

  float Current = ShuntVoltage * 0.1;
  if (gpio_get(26) != 0) {
    Current = -Current;
  }
  filteredCurrent = filter.solve(Current);

  motor1_output = motor1_pid.solve(targetCurrent - filteredCurrent);
  motor1.setOutput(motor1_output);

  motor1.update();

  delay(1);
}

void setup1() {
}

void loop1() {
  Serial.print(targetCurrent);
  Serial.print(",");
  Serial.print(filteredCurrent);
  Serial.print(",");
  Serial.print(motor1_output);
  Serial.print(",");
  Serial.print(motor1_pid.getIntegral());
  Serial.print(",");
  Serial.print(BusVoltage);
  Serial.println();

  delay(1);
}
