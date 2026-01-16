#include <Nidec_24H_BLDC_Pico.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TinyUSB.h>
#include <math.h>

// NeoPixel WS2812B
#define NUM_LEDS 3
#define LED_PIN 15
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motors
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };
NidecBLDC motor2{ 7, 8, 9, 10, 11, 1.0f };
NidecBLDC motor3{ 16, 17, 18, 19, 20, 20.0f };
NidecBLDC motor4{ 21, 22, 26, 27, 28, 40.0f };

NidecBLDC* motors[4] = { &motor1, &motor2, &motor3, &motor4 };

// USB MIDI
Adafruit_USBD_MIDI usb_midi;

// Voice Management
struct Voice {
  bool active;
  uint8_t note;
  float frequency;
  int16_t amplitude;
  unsigned long halfPeriodMicros;
  unsigned long lastToggleTime;
  bool outputState;
};

Voice voices[4];

void noteOn(uint8_t note, uint8_t velocity) {
  // 0 velocity is Note Off
  if (velocity == 0) {
    noteOff(note, velocity);
    return;
  }

  // Find free voice
  int voiceIdx = -1;
  for (int i = 0; i < 4; i++) {
    if (!voices[i].active) {
      voiceIdx = i;
      break;
    }
  }

  // If no free voice, steal one (simple round-robin or first found)
  if (voiceIdx == -1) {
    voiceIdx = 0;  // Simple stealing
  }

  // Calculate params
  voices[voiceIdx].active = true;
  voices[voiceIdx].note = note;
  voices[voiceIdx].amplitude = map(velocity, 0, 127, 0, 32767);
  voices[voiceIdx].frequency = 440.0f * pow(2.0f, (note - 69) / 12.0f);

  if (voices[voiceIdx].frequency > 0) {
    voices[voiceIdx].halfPeriodMicros = (unsigned long)(1000000.0f / voices[voiceIdx].frequency / 2.0f);
  } else {
    voices[voiceIdx].halfPeriodMicros = 0;
  }

  voices[voiceIdx].lastToggleTime = micros();
  voices[voiceIdx].outputState = false;

  // Visual feedback
  if (voiceIdx < NUM_LEDS) {
    // Hue based on Note, Value based on Velocity
    uint32_t color = strip.ColorHSV((uint16_t)note * 512, 255, (uint8_t)map(velocity, 0, 127, 0, 255));
    strip.setPixelColor(voiceIdx, color);
    strip.show();
  }
}

void noteOff(uint8_t note, uint8_t velocity) {
  for (int i = 0; i < 4; i++) {
    if (voices[i].active && voices[i].note == note) {
      voices[i].active = false;
      voices[i].amplitude = 0;
      motors[i]->setOutput(0);

      if (i < NUM_LEDS) {
        strip.setPixelColor(i, 0);
        strip.show();
      }
    }
  }
}

void setup() {
  strip.begin();
  strip.show();
  strip.setBrightness(255);

  Serial.begin(115200);

  // Initialize MIDI
  usb_midi.setStringDescriptor("Pico Nidec Synthesizer");
  usb_midi.begin();

  delay(1000);

  for (int i = 0; i < 4; i++) {
    if (!motors[i]->begin()) {
      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.println(" Error");
    }
    motors[i]->setZeroPoint();
    motors[i]->setOutput(0);
  }
}

void loop() {
  // --- Handle USB MIDI ---
  uint8_t packet[4];
  while (usb_midi.readPacket(packet)) {
    uint8_t codeIndex = packet[0] & 0x0F;
    uint8_t byte1 = packet[1];
    uint8_t byte2 = packet[2];
    uint8_t byte3 = packet[3];

    // Note On
    if (codeIndex == 0x9) {
      noteOn(byte2, byte3);
    }
    // Note Off
    else if (codeIndex == 0x8) {
      noteOff(byte2, byte3);
    }
  }

  // --- Sound Generation ---
  unsigned long currentMicros = micros();

  for (int i = 0; i < 4; i++) {
    motors[i]->update();

    if (voices[i].active && voices[i].halfPeriodMicros > 0) {
      if (currentMicros - voices[i].lastToggleTime >= voices[i].halfPeriodMicros) {
        voices[i].outputState = !voices[i].outputState;
        voices[i].lastToggleTime = currentMicros;

        int16_t out = voices[i].outputState ? voices[i].amplitude : -voices[i].amplitude;
        motors[i]->setOutput(out);
      }
    } else {
      // Ensure silence if inactive
      // motors[i]->setOutput(0); // Already set in NoteOff, but good for safety if needed
    }
  }
}
