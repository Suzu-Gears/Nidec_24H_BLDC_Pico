#include <RP2040PIO_CAN.h>

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

void setup() {
  Serial.begin(115200);

  CAN.setRX(CAN_RX_PIN);
  CAN.setTX(CAN_TX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
}

void loop() {
  uint32_t now = millis();  // 32bit

  CanMsg msg;
  msg.id = CanStandardId(0x01);
  msg.data_length = 8;
  // 32bitを8bit x 4に分割して、リトルエンディアンで格納
  msg.data[0] = now >> 24;
  msg.data[1] = now >> 16;
  msg.data[2] = now >> 8;
  msg.data[3] = now;
  // 残りは0で埋まっているのでそのまま
  CAN.write(msg);

  Serial.println("Sent CAN message with timestamp: " + String(now) + " ms");
  delay(100);
}
