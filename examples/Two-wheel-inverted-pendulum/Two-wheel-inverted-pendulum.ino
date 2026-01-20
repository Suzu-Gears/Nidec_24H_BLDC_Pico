#include <RP2040PIO_CAN.h>  // For RP2040, RP2350, etc.
#include <DAMIAO_IMU_CAN.h>
#include <Nidec_24H_BLDC_Pico.h>
#include "Nidec_Robomas.h"
#include "low_pass.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;
const uint8_t IMU_MASTER_ID = 0x02;
const uint8_t IMU_SLAVE_ID = 0x01;

constexpr float wheel_radius = 0.070f;

LowPassFilter filter;

DamiaoImuCan imu(IMU_MASTER_ID, IMU_SLAVE_ID, &CAN1);
NidecBLDC motor1{ 2, 3, 4, 5, 6, 10.0f };
NidecBLDC motor2{ 7, 8, 9, 10, 11, 10.0f };

NidecBLDC* motors[] = { &motor1, &motor2 };

Note RobomasMelody[] = {
  { 0, 0.00, 1000 },
  { 32767, 533.0, 240 },
  { 32767, 605.0, 240 },
  { 32767, 795.0, 350 },
  { 0, 0.00, 1000 }
};

void setup() {
  Serial.begin(115200);

  bool can_ok = false;
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

  filter.init(0.01);

  motor1.setZeroPoint();
  motor2.setZeroPoint();
}

void loop() {
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
  float speed = motor1.getRps() * wheel_radius;

  float output = (roll_rad * 180000.0f) + (gyro_x * 10000.0f) + (position * 1000.0f) + (-speed * 6000.0f);
  float filtered_output = filter.solve(output);
  motor1.setOutput(filtered_output);
  motor2.setOutput(-filtered_output);

  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {  // 100msごとに表示
    last_print_time = millis();
    Serial.print(roll_rad, 4);
    Serial.print(", ");
    Serial.print(gyro_x, 4);
    Serial.print(", ");
    Serial.print(wheel1_rad, 4);
    Serial.print(", ");
    Serial.print(position, 4);
    Serial.print(", ");
    Serial.print(speed, 4);
    Serial.println();
  }
  delay(1);
}
