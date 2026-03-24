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
  while (CAN.available()) {
    CanMsg msg = CAN.read();

    Serial.print(" ID: 0x");
    char id_str[5];
    sprintf(id_str, "%03X", msg.getStandardId());
    Serial.print(id_str);
    Serial.print(" Data: ");
    for (int i = 0; i < msg.data_length; i++) {
      char data_str[3];
      sprintf(data_str, "%02X", msg.data[i]);
      Serial.print("0x");
      Serial.print(data_str);
      Serial.print(" ");
    }

    Serial.print("Received little-endian value: ");
    uint32_t value = static_cast<uint32_t>(msg.data[0] << 24)
                     | static_cast<uint32_t>(msg.data[1] << 16)
                     | static_cast<uint32_t>(msg.data[2] << 8)
                     | static_cast<uint32_t>(msg.data[3]);
    Serial.print(value);

    Serial.println();
  }
}
