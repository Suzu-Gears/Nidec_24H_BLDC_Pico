// https://github.com/sanorobo/catchrobo2025_third_motor/blob/master/NidecBLDC.h

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Arduino.h>

#include <PicoEncoder.h>

class NidecBLDC {
public:
  NidecBLDC(pin_size_t brake_pin, pin_size_t direction_pin, pin_size_t pwm_pin,
            pin_size_t encoder_a_pin, float reduction_ratio = 1.0f)
    : brake_pin_{ brake_pin }, direction_pin_{ direction_pin }, pwm_pin_{ pwm_pin },
      encoder_a_pin_{ encoder_a_pin }, reduction_ratio_{ reduction_ratio } {}

  bool begin() {
    pinMode(brake_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);
    analogWriteFreq(PWM_FREQ);
    analogWriteResolution(15);

    if (encoder_.begin(encoder_a_pin_) != 0) {
      return false;
    }

    setOutput(0.0f);

    return true;
  }

  void update() {
    encoder_.update();

    if (std::abs(getRps()) > NONZERO_THRESHOLD) {
      last_nonzero_ = millis();
    }

    if (millis() - last_nonzero_ > NONZERO_TIMEOUT) {
      brakeOn();
      delayMicroseconds(10);
      brakeOff();
    }
  }

  float getRps() {
    return encoder_.speed / 64.0f / ENCODER_PPR / reduction_ratio_;
  }

  float getRpm() {
    return getRps() * 60.0f;
  }

  float getRad() {
    return encoder_.position / 64.0f / ENCODER_PPR * 2 * M_PI / reduction_ratio_;
  }

  void setZeroPoint() {
    encoder_.resetPosition();
  }

  // -32767 ~ 32767
  void setOutput(int16_t output) {
    if (output < 0) {
      digitalWrite(direction_pin_, LOW);
    } else {
      digitalWrite(direction_pin_, HIGH);
    }
    analogWrite(pwm_pin_, std::clamp<int>(32767 - std::abs(output), 0, 32767));
  }

private:
  static constexpr uint32_t ENCODER_PPR = 400;
  static constexpr uint32_t PWM_FREQ = 30000;
  static constexpr float NONZERO_THRESHOLD = 0.01f;
  static constexpr uint32_t NONZERO_TIMEOUT = 10;

  pin_size_t brake_pin_;
  pin_size_t direction_pin_;
  pin_size_t pwm_pin_;
  pin_size_t encoder_a_pin_;
  float reduction_ratio_;
  unsigned long last_nonzero_ = 0;
  PicoEncoder encoder_;

  void brakeOn() {
    digitalWrite(brake_pin_, LOW);
  }

  void brakeOff() {
    digitalWrite(brake_pin_, HIGH);
  }
};
