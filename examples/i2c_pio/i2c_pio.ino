#include <Arduino.h>  // Arduinoフレームワークのヘッダ
#include "hardware/pio.h"
#include "hardware/clocks.h"

// CのソースファイルをC++から呼び出すためのextern "C"
extern "C" {
#include "pio_i2c.h"
}
#include "i2c.pio.h"


const uint PIN_SDA = 4;
const uint PIN_SCL = 5;  // SCL must be SDA + 1 (for wait mapping)
const uint8_t INA219_ADDR = 0x40;

PIO pio = pio0;
uint sm = 0;  // ステートマシンは0番を使う

void setup() {
  Serial.begin(115200);
  delay(2000);  // シリアルモニタが開くのを待つ
  Serial.println("PIO I2C with INA219 (Official Example Base)");

  // PIO I2Cの初期化 (公式サンプル版)
  // i2c.pio.h で定義されている
  uint offset = pio_add_program(pio, &i2c_program);
  i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);

  // --- INA219のコンフィグレーションレジスタ(0x00)を設定 ---
  // 公式サンプルの pio_i2c_write_blocking は、
  // 送信バッファの先頭に「書き込みたいレジスタのアドレス」を含める必要がある
  uint16_t config_value = 0x399F;
  uint8_t write_buf[3];
  write_buf[0] = 0x00;                            // 1バイト目: 書き込み先レジスタのアドレス (Config Register)
  write_buf[1] = (uint8_t)(config_value >> 8);    // 2バイト目: データ上位バイト
  write_buf[2] = (uint8_t)(config_value & 0xFF);  // 3バイト目: データ下位バイト

  Serial.println("Writing INA219 Config...");
  pio_i2c_write_blocking(pio, sm, INA219_ADDR, write_buf, 3);
  Serial.println("Config Written.");

  delay(100);
}

void loop() {
  uint8_t reg_addr_to_read;
  uint8_t read_buf[2] = { 0, 0 };

  // --- Bus Voltage (0x02) の読み出し ---
  // 1. 読み出したいレジスタのアドレス(0x02)をINA219に送信する
  reg_addr_to_read = 0x02;
  pio_i2c_write_blocking(pio, sm, INA219_ADDR, &reg_addr_to_read, 1);

  // 2. INA219から2バイトのデータを読み出す
  pio_i2c_read_blocking(pio, sm, INA219_ADDR, read_buf, 2);

  uint16_t rawBusVoltage = (uint16_t)(read_buf[0] << 8 | read_buf[1]);
  float busVoltage = (float)((rawBusVoltage >> 3) * 4);

  // --- Shunt Voltage (0x01) の読み出し ---
  // 1. 読み出したいレジスタのアドレス(0x01)をINA219に送信する
  reg_addr_to_read = 0x01;
  pio_i2c_write_blocking(pio, sm, INA219_ADDR, &reg_addr_to_read, 1);

  // 2. INA219から2バイトのデータを読み出す
  pio_i2c_read_blocking(pio, sm, INA219_ADDR, read_buf, 2);

  int16_t rawShuntVoltage = (int16_t)(read_buf[0] << 8 | read_buf[1]);
  float current = rawShuntVoltage * 0.1;
  float shuntVoltage = rawShuntVoltage * 10.0;

  // 結果をシリアルモニタに出力
  Serial.print("Bus Voltage: ");
  Serial.print(busVoltage);
  Serial.print(" mV, ");
  Serial.print("Shunt Voltage: ");
  Serial.print(shuntVoltage);
  Serial.print(" uV, ");
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" mA");

  delay(1000);
}
