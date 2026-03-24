#include <RP2040PIO_CAN.h>  // For RP2040, RP2350, etc.
#include <DAMIAO_IMU_CAN.h>
#include <Nidec_24H_BLDC_Pico.h>
#include "Nidec_Robomas.h"
#include "pid_controller.hpp"
#include "low_pass.h"
#include "PS3.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;
const uint32_t SBDBT_TX_PIN = 12;
const uint32_t SBDBT_RX_PIN = 13;
const uint8_t IMU_MASTER_ID = 0x02;
const uint8_t IMU_SLAVE_ID = 0x01;

constexpr float wheel_radius = 0.035f;

LowPassFilter leftStickFilter;
LowPassFilter rightStickFilter;
PidController position_pid, speed_pid;
PS3 sbdpico;

DamiaoImuCan imu(IMU_MASTER_ID, IMU_SLAVE_ID, &CAN1);
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };

NidecBLDC* motors[] = { &motor1, &motor2 };

Note RobomasMelody[] = {
  { 0, 0.00, 1000 },
  { 32767, 533.0, 240 },
  { 32767, 605.0, 240 },
  { 32767, 795.0, 350 },
  { 0, 0.00, 1000 }
};

// Tunable Gains
float gain_angle = 180000.0f;
float gain_gyro_x = 6000.0f;
float gain_gyro_z = 1000.0f;
float gain_position_KP = 0.2f;
float gain_position_KI = 0.01f;
float gain_position_KD = 0.1f;
float gain_speed_KP = 0.5f;
float gain_speed_KI = 0.0f;
float gain_yaw = 10000.0f;
float target_yaw = 0.0f;  // IMUの初回読込時のyawを代入すべきかも
float target_angle = 0;
float target_position = 0.0f;

float getRollFromQuaternion_rad() {
  float qw = imu.getQuatW();
  float qx = imu.getQuatX();
  float qy = imu.getQuatY();
  float qz = imu.getQuatZ();

  float gravity_y = 2.0f * (qy * qz + qw * qx);
  float gravity_z = qw * qw - qx * qx - qy * qy + qz * qz;
  float quat_roll_rad = atan2(gravity_y, gravity_z);

  return quat_roll_rad;
}

float getYawFromQuaternion_rad() {
  float qw = imu.getQuatW();
  float qx = imu.getQuatX();
  float qy = imu.getQuatY();
  float qz = imu.getQuatZ();

  float gravity_x = 2.0f * (qx * qy + qw * qz);
  float gravity_y = qw * qw - qx * qx + qy * qy - qz * qz;
  float quat_yaw_rad = atan2(gravity_x, gravity_y);

  return quat_yaw_rad;
}

float normalizeAngle(float rad) {
  while (rad > PI) rad -= 2.0f * PI;
  while (rad < -PI) rad += 2.0f * PI;
  return rad;
}

void setup() {
  Serial.begin(115200);

  Serial1.setRX(SBDBT_RX_PIN);
  Serial1.setTX(SBDBT_TX_PIN);
  Serial1.begin(115200);
  sbdpico.setSerial(&Serial1);

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

  leftStickFilter.init(0.5f);
  rightStickFilter.init(0.5f);
  position_pid.init({ 0.001f, gain_position_KP, gain_position_KI, 0.0f, 0.0f, 0.3f, 10.0f });
  //speed_pid.init({ 0.001f, gain_speed_KP, gain_speed_KI, 0.0f, 0.0f, 1.0f, 1.0f });

  motor1.setZeroPoint();
  motor2.setZeroPoint();
}

void loop() {
  // Parse Serial Input for Gain Tuning
  // Format: "a1000" -> Angle Gain = 1000
  //         "g500"  -> Gyro Gain = 500
  //         "p100"  -> Position Gain = 100
  //         "s200"  -> Speed Gain = 200
  //         "y5000" -> Yaw Gain = 5000
  if (Serial.available()) {
    char key = Serial.read();
    // Check if the key is valid before parsing the float
    if (key == 'a' || key == 'g' || key == 'p' || key == 'd' || key == 's' || key == 'y' || key == 'z') {
      float val = Serial.parseFloat();

      switch (key) {
        case 'a': gain_angle = val; break;
        case 'g': gain_gyro_x = val; break;
        case 'p': gain_position_KP = val; break;
        case 'd': gain_position_KD = val; break;
        case 's': gain_speed_KP = val; break;
        case 'z': gain_gyro_z = val; break;
        case 'y': gain_yaw = val; break;
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
  motor2.update();
  imu.update();
  sbdpico.update();

  float rawLeftY = sbdpico.getAxis(PS3Axis::LEFT_Y);    // -1.0~1.0
  float rawRightX = sbdpico.getAxis(PS3Axis::RIGHT_X);  // -1.0~1.0

  float filteredLeftY = leftStickFilter.solve(rawLeftY);
  float filteredRightX = rightStickFilter.solve(rawRightX);

  constexpr float offset_rad = 0.035f;
  float roll_rad = getRollFromQuaternion_rad() - offset_rad;

  float avg_wheel_rad = (motor1.getRad() + (-1.0f) * motor2.getRad()) / 2.0f;
  float position = (roll_rad + avg_wheel_rad) * wheel_radius;
  float avg_wheel_speed = (motor1.getVelocity() + (-1.0f) * motor2.getVelocity()) / 2.0f;
  float speed = avg_wheel_speed * wheel_radius;

  // 位置制御ベースの目標位置更新
  // スティックの傾け量に応じて目標位置を現在位置から離していく
  target_position += (-1.0f) * filteredLeftY * 0.0015f;                             // スピード係数は調整が必要 0.001だとゆっくりだけど安定
  target_position = std::clamp(target_position, position - 0.5f, position + 0.5f);  // 目標位置は現在位置から離しすぎない
  //float ff_speed = (-1.0f) * filteredLeftY * 0.1f;       // フィードフォワードの速度成分（位置制御の目標速度としても利用）
  float ff_speed = 0.0f;

  // 位置PD制御（位置ロック）
  target_angle = (target_position - position) * gain_position_KP + (0.0f - speed) * gain_position_KD + ff_speed;
  target_angle = std::clamp(target_angle, -0.3f, 0.3f);

  float error_angle = (target_angle - roll_rad);
  float gyro_x = imu.getGyroX();
  float balance = -error_angle * gain_angle + gyro_x * gain_gyro_x;

  // target_yawの更新 (右スティック右で時計回り/yaw減少方向へ)
  target_yaw -= filteredRightX * 0.0025f;
  target_yaw = normalizeAngle(target_yaw);                     // 目標値は常に正規化された角度を保持する (-PI ~ PI)
  float current_yaw = getYawFromQuaternion_rad();              // 現在のyaw (-PI ~ PI)
  float error_yaw = normalizeAngle(target_yaw - current_yaw);  // 最短距離で回転するための正規化処理 (-PI ~ PI)
  float gyro_z = imu.getGyroZ();

  float turn = (-1.0f) * error_yaw * gain_yaw + gyro_z * gain_gyro_z;  // 角度誤差とyaw速度の両方を考慮

  float output1 = std::clamp(balance - turn, -32767.0f, 32767.0f);
  float output2 = std::clamp(balance + turn, -32767.0f, 32767.0f);

  // 安全機構(転倒時 or 丸ボタン押下時は出力0)
  if (abs(roll_rad) > 1.0f || sbdpico.getKey(PS3Key::CIRCLE)) {
    target_yaw = current_yaw;    // 転倒後は現在のyawを目標にする
    target_position = position;  // 転倒後は現在の位置を目標にする
    output1 = 0;
    output2 = 0;
  }

  motor1.setOutput(output1);
  motor2.setOutput((-1.0f) * output2);

  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {  // 100msごとに表示
    last_print_time = millis();

    Serial.print("TargetAngle:");
    Serial.print(target_angle, 4);
    Serial.print(" | Gains A:");
    Serial.print(gain_angle, 0);
    Serial.print(" Gx:");
    Serial.print(gain_gyro_x, 0);
    Serial.print(" P:");
    Serial.print(gain_position_KP, 4);
    Serial.print(" S:");
    Serial.print(gain_speed_KP, 4);
    Serial.print(" D:");
    Serial.print(gain_position_KD, 4);
    Serial.print(" Y:");
    Serial.print(gain_yaw, 0);
    Serial.print(" Gz:");
    Serial.print(gain_gyro_z, 0);

    Serial.print(" | AngleRad:");
    Serial.print(roll_rad, 4);
    Serial.print(", GyroX:");
    Serial.print(gyro_x, 4);
    Serial.print(", target_position:");
    Serial.print(target_position, 4);
    Serial.print(", Position:");
    Serial.print(position, 4);
    Serial.print(", Speed: ");
    Serial.print(speed);

    Serial.print(" | YawRad:");
    Serial.print(current_yaw, 4);
    Serial.print(", TargetYaw:");
    Serial.print(target_yaw, 4);

    Serial.print(" | Output:");
    Serial.print(output1, 4);
    Serial.println();
  }
  delay(1);
}
