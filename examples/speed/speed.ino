#define idle_stop_samples 100  // PicoEncoderの速度が0とみなすサンプル数の閾値を100に変更して低速時でも

#include <Nidec_24H_BLDC_Pico.h>

NidecBLDC motor{ 28, 27, 26, 15, 14, 1.0f };

void setup() {
  Serial.begin(115200);

  motor.begin();
}

void loop() {
  static unsigned long last_print_time = 0;
  if (micros() - last_print_time > 1000) {  // 1000usごとに実行(1kHz)
    last_print_time = micros();

    motor.update();

    int position = motor.getRawPosition();
    int speed = motor.getRawSpeed();

    static int previous_position = 0;
    int delta_position = position - previous_position;
    float calculated_speed = delta_position / 0.001f;  // 1msごとに位置の変化から速度を計算
    previous_position = position;

    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" | Speed: ");
    Serial.print(speed);
    Serial.print(" | Calculated Speed: ");
    Serial.print(calculated_speed);
    Serial.println();
  }
}
