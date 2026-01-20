#pragma once

#include "Nidec_24H_BLDC_Pico.h"

struct Note {
  int16_t amplitude;  // Output absolute value (0-32767)
  float frequency;    // Hz
  uint32_t duration;  // ms
};

class NidecSoundPlayer {
public:
  /**
   * @brief Plays a melody on multiple motors synchronously (Blocking).
   *
   * @param motors Array of pointers to NidecBLDC objects.
   * @param motorCount Number of motors in the array.
   * @param melody Array of Note structures representing the melody.
   * @param melodyLength Number of notes in the melody array.
   */
  static void playMelody(NidecBLDC** motors, size_t motorCount, const Note* melody, size_t melodyLength) {
    for (size_t i = 0; i < melodyLength; i++) {
      playNote(motors, motorCount, melody[i]);
    }
    // Ensure motors are stopped after the melody finishes
    stopMotors(motors, motorCount);
  }

private:
  static void playNote(NidecBLDC** motors, size_t motorCount, Note note) {
    unsigned long startTime = millis();
    unsigned long lastToggleMicros = micros();
    bool outputState = false;
    unsigned long halfPeriod = 0;

    if (note.frequency > 0) {
      halfPeriod = (unsigned long)(1000000.0f / note.frequency / 2.0f);
    }

    // Play the note for its duration
    while (millis() - startTime < note.duration) {
      unsigned long currentMicros = micros();

      // Call update() to keep internal motor states managed if necessary
      for (size_t m = 0; m < motorCount; m++) {
        motors[m]->update();
      }

      if (note.frequency > 0 && note.amplitude > 0) {
        // Generate square wave
        if (currentMicros - lastToggleMicros >= halfPeriod) {
          outputState = !outputState;
          lastToggleMicros = currentMicros;
          int16_t pwm = outputState ? note.amplitude : -note.amplitude;

          for (size_t m = 0; m < motorCount; m++) {
            motors[m]->setOutput(pwm);
          }
        }
      } else {
        // Rest (Silence)
        for (size_t m = 0; m < motorCount; m++) {
          motors[m]->setOutput(0);
        }
      }
    }
  }

  static void stopMotors(NidecBLDC** motors, size_t motorCount) {
    for (size_t m = 0; m < motorCount; m++) {
      motors[m]->setOutput(0);
    }
  }
};
