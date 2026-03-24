#include <RP2040PIO_CAN.h>  // For RP2040, RP2350, etc.
#include <DAMIAO_IMU_CAN.h>
#include <Nidec_24H_BLDC_Pico.h>
#include "Nidec_Robomas.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;
const uint8_t IMU_MASTER_ID = 0x02;
const uint8_t IMU_SLAVE_ID = 0x01;

constexpr float wheel_radius = 0.070f;

DamiaoImuCan imu(IMU_MASTER_ID, IMU_SLAVE_ID, &CAN1);
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };

NidecBLDC* motors[] = { &motor1, &motor2 };

// Tunable Gains
float gain_angle = 180000.0f;
float gain_gyro = 10000.0f;
float gain_position = 0.0f;
float gain_speed = 0.0f;

Note RobomasMelody[] = {
  { 0, 0.00, 1000 },
  { 32767, 533.0, 240 },
  { 32767, 605.0, 240 },
  { 32767, 795.0, 350 },
  { 0, 0.00, 1000 }
};

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);  // Short timeout to prevent blocking control loop

  CAN1.setTX(CAN_TX_PIN);
  CAN1.setRX(CAN_RX_PIN);

  if (!CAN1.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
  Serial.println("CAN bus initialized.");

  // IMUをアクティブモードに設定
  if (imu.changeToActive()) {
    Serial.println("IMU successfully changed to Active Mode.");
  } else {
    Serial.println("Failed to change IMU to Active Mode!");
  }

  if (!motor1.begin()) {
    Serial.println("Motor 1 Encoder Error");
    while (1);
  }
  if (!motor2.begin()) {
    Serial.println("Motor 2 Encoder Error");
    while (1);
  }

  NidecSoundPlayer::playMelody(motors, 2, RobomasMelody, sizeof(RobomasMelody) / sizeof(Note));

  motor1.setZeroPoint();
  motor2.setZeroPoint();
}

void loop() {
  // Parse Serial Input for Gain Tuning
  // Format: "a1000" -> Angle Gain = 1000
  //         "g500"  -> Gyro Gain = 500
  //         "p100"  -> Position Gain = 100
  //         "s200"  -> Speed Gain = 200
  if (Serial.available()) {
    char key = Serial.read();
    // Check if the key is valid before parsing the float
    if (key == 'a' || key == 'g' || key == 'p' || key == 's') {
      float val = Serial.parseFloat();

      switch (key) {
        case 'a': gain_angle = val; break;
        case 'g': gain_gyro = val; break;
        case 'p': gain_position = val; break;
        case 's': gain_speed = val; break;
      }

      // Clear buffer
      while (Serial.available() > 0) Serial.read();

      Serial.print("Updated Gain ");
      Serial.print(key);
      Serial.print(" to: ");
      Serial.println(val);
    }
  }
  motor1.update();
  imu.update();
  float gyro_x = imu.getGyroX();
  float roll = imu.getRoll() + 360.0f;  // 0.0 ~ 360.0 deg
  if (roll > 180.0f) {
    roll -= 360.0f;  // -180.0 ~ 180.0 deg
  }
  constexpr float offset_rad = 0.035f;
  float roll_rad = roll * PI / 180.0f - offset_rad;

  float wheel1_rad = motor1.getRad();
  float position = (roll_rad + wheel1_rad) * wheel_radius;

  static bool first_loop = true;
  static float last_position = 0.0f;
  static unsigned long last_calc_time = 0;

  unsigned long current_time = micros();
  float speed = 0.0f;

  if (first_loop) {
    last_position = position;
    last_calc_time = current_time;
    first_loop = false;
  } else {
    float dt = (current_time - last_calc_time) / 1000000.0f;
    if (dt > 1e-5f) {
      speed = (position - last_position) / dt;
      last_position = position;
      last_calc_time = current_time;
    }
  }

  // Use global gain variables
  float term_angle = roll_rad * gain_angle;
  float term_gyro = gyro_x * gain_gyro;
  float term_position = position * gain_position;
  float term_speed = -speed * gain_speed;

  float output = term_angle + term_gyro + term_position + term_speed;
  motor1.setOutput(output);
  motor2.setOutput(-output);

  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {  // 100msごとに表示
    last_print_time = millis();

    float total_abs = abs(term_angle) + abs(term_gyro) + abs(term_position) + abs(term_speed);
    if (total_abs < 1e-6) total_abs = 1.0f;  // Avoid division by zero

    Serial.print("Angle: ");
    Serial.print(abs(term_angle) / total_abs * 100.0f, 1);
    Serial.print("% Gyro: ");
    Serial.print(abs(term_gyro) / total_abs * 100.0f, 1);
    Serial.print("% Pos: ");
    Serial.print(abs(term_position) / total_abs * 100.0f, 1);
    Serial.print("% Speed: ");
    Serial.print(abs(term_speed) / total_abs * 100.0f, 1);

    Serial.print("% | Gains A:");
    Serial.print(gain_angle, 0);
    Serial.print(" G:");
    Serial.print(gain_gyro, 0);
    Serial.print(" P:");
    Serial.print(gain_position, 0);
    Serial.print(" S:");
    Serial.print(gain_speed, 0);

    Serial.print(" | AngleRad:");
    Serial.print(roll_rad, 4);
    Serial.print(", GyroX:");
    Serial.print(gyro_x, 4);
    Serial.print(", Position:");
    Serial.print(position, 4);
    Serial.print(", Speed: ");
    Serial.print(speed);
    Serial.println();
  }
  delay(1);
}
