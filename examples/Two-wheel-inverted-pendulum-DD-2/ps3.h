#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <Arduino.h>

enum class PS3Axis {
  LEFT_X,
  LEFT_Y,
  RIGHT_X,
  RIGHT_Y,
};

enum class PS3Key {
  UP,
  DOWN,
  RIGHT,
  LEFT,
  TRIANGLE,
  CROSS,
  CIRCLE,
  SQUARE = 8,
  L1,
  L2,
  R1,
  R2,
  START,
  SELECT,
};

class PS3 {
public:
  void setSerial(arduino::HardwareSerial *uart) {
    uart_ = uart;
  }

  void update() {
    keys_prev_ = keys_;

    while (receiveMessage()) {
      if (testChecksum(buf_)) {
        keys_ = (buf_[1] << 8) | buf_[2];
        if ((keys_ & 0x03) == 0x03) {
          keys_ &= ~0x03;
          keys_ |= 1 << 13;
        }
        if ((keys_ & 0x0C) == 0x0C) {
          keys_ &= ~0x0C;
          keys_ |= 1 << 14;
        }
        for (size_t i = 0; i < 4; ++i) {
          axes_[i] = (static_cast<float>(buf_[i + 3]) - 64) / 64;
        }
      }
      buf_.fill(0);
    }
  }

  float getAxis(PS3Axis axis) {
    return axes_[toUnderlying(axis)];
  }

  bool getKey(PS3Key key) {
    return (keys_ & (1 << toUnderlying(key))) != 0;
  }

  bool getKeyDown(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_ & (1 << toUnderlying(key))) != 0;
  }

  bool getKeyUp(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_prev_ & (1 << toUnderlying(key))) != 0;
  }

private:
  arduino::HardwareSerial *uart_;
  std::array<uint8_t, 8> buf_{};
  std::array<float, 4> axes_{};
  uint16_t keys_ = 0;
  uint16_t keys_prev_ = 0;

  bool receiveMessage() {
    // ヘッダ探す
    for (size_t i = 0; i < 8; ++i) {
      if (buf_[0] == 0x80) {
        break;
      }
      if (uart_->available() < 1) {
        return false;
      }
      buf_[0] = uart_->read();
    }
    if (buf_[0] != 0x80) {
      return false;
    }

    // メッセージの残りの部分を受信
    if (uart_->available() < 7) {
      return false;
    }
    uart_->readBytes(&buf_[1], 7);
    return true;
  }

  static inline bool testChecksum(const std::array<uint8_t, 8> &msg) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < 7; ++i) {
      checksum += msg[i];
    }
    return (checksum & 0x7F) == msg[7];
  }

  template<class T> constexpr std::underlying_type_t<T> toUnderlying(T value) {
    return static_cast<std::underlying_type_t<T>>(value);
  }
};
