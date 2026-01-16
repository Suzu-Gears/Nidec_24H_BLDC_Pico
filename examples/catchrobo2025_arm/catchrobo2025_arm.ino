// https://github.com/sanorobo/catchrobo2025_third_motor/blob/master/catchrobo2025_third_motor.ino

#include <cstdint>
#include <cstring>

#include <RP2040PIO_CAN.h>

#include "NidecBLDC.h"

#define MOTOR_ID 2

#if MOTOR_ID == 0
NidecBLDC motor{ 15, 26, 14, 27, 20.0f };
#elif MOTOR_ID == 1
NidecBLDC motor{ 15, 26, 14, 27, 100.0f };
#elif MOTOR_ID == 2
// NidecBLDC motor{ 15, 26, 14, 27, 100.0f };
NidecBLDC motor{ 14, 26, 15, 27, 20.0f };
#endif

void setup() {
  Serial.begin(115200);

  // CAN
  CAN1.setTX(0);
  CAN1.setRX(1);
  CAN1.begin(CanBitRate::BR_1000k);

  // Motor
  motor.begin();
}

void loop() {
  motor.update();

  // 指令受信
  while (CAN1.available()) {
    CanMsg msg = CAN1.read();
    if (!msg.isStandardId()) {
      continue;
    }
    uint32_t id = msg.getStandardId();
    if (id == 0x01 && msg.data_length == 8) {
      int16_t output;
      std::memcpy(&output, &msg.data[MOTOR_ID * 2], 2);
      motor.setOutput(output);
    } else if (id == 0x0F && msg.data_length == 0) {
      motor.setZeroPoint();
    }
  }

  // フィードバック送信
  float rps = motor.getRps();
  float rad = motor.getRad();
  uint8_t data[8];
  std::memcpy(&data[0], &rps, 4);
  std::memcpy(&data[4], &rad, 4);
  CanMsg msg(CanStandardId(0x10 + MOTOR_ID), 8, data);
  CAN1.write(msg);

  delay(1);
}
