#include <RP2040PIO_CAN.h>

#define DEVICE_ID 0x01  // 自分のIDは読まずに破棄するので、書き込むボードごとに番号を分ける

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

void setup() {
  Serial.begin(115200);

  CAN.setRX(CAN_RX_PIN);
  CAN.setTX(CAN_TX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
}

void loop() {
  static uint32_t received_value = 0;

  // 受信処理
  while (CAN.available()) {  // 受信できるメッセージがあれば全部読む
    CanMsg rx_msg = CAN.read();

    if (rx_msg.getStandardId() != DEVICE_ID) {  // 自分以外のメッセージを読む
      received_value = static_cast<uint32_t>(rx_msg.data[0] << 24)
                       | static_cast<uint32_t>(rx_msg.data[1] << 16)
                       | static_cast<uint32_t>(rx_msg.data[2] << 8)
                       | static_cast<uint32_t>(rx_msg.data[3]);
    }
  }

  uint32_t now = millis();

  // 送信処理
  CanMsg tx_msg;
  tx_msg.id = CanStandardId(DEVICE_ID);
  tx_msg.data_length = 8;
  tx_msg.data[0] = now >> 24;
  tx_msg.data[1] = now >> 16;
  tx_msg.data[2] = now >> 8;
  tx_msg.data[3] = now;
  CAN.write(tx_msg);

  // 情報の表示
  Serial.print("Sent value: ");
  Serial.print(now);
  Serial.print(" Last received value: ");
  Serial.println(received_value);

  delay(100);
}
