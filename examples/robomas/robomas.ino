#include <Nidec_24H_BLDC_Pico.h>
#include <Adafruit_NeoPixel.h>

// NeoPixel WS2812Bの指定
#define NUM_LEDS 3  // NeoPixelの数
#define LED_PIN 15  // NeoPixel信号接続端子
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// encoder_a, encoder_b, direction, pwm_pin, brake, reduction_ratio)
// Motor 1: Sound Source (制御対象)
NidecBLDC motor1{ 2, 3, 4, 5, 6, 1.0f };

struct Note {
  int16_t amplitude;  // Output absolute value (0-32767)
  float frequency;    // Hz
  uint32_t duration;  // ms
};

// 再生リスト
Note melody[] = {
  // { 15000, 261.63, 500 },   // C4
  // { 15000, 293.66, 500 },   // D4
  // { 15000, 329.63, 500 },   // E4
  // { 15000, 349.23, 500 },   // F4
  // { 15000, 392.00, 500 },   // G4
  // { 15000, 440.00, 500 },   // A4
  // { 15000, 493.88, 500 },   // B4
  // { 15000, 523.25, 1000 },  // C5
  { 0, 0.00, 1000 },  // Rest
  { 32767, 533.0, 240 },
  { 32767, 605.0, 240 },
  { 32767, 795.0, 350 },
  { 0, 0.00, 1000 }  // Rest
};

void setup() {
  strip.begin();                                  // NeoPixel初期化
  strip.show();                                   // NeoPixelを全て消灯
  strip.setBrightness(255);                       // 明るさ倍率設定 (RGB設定値 * 明るさ倍率、最大255)
  strip.setPixelColor(0, strip.Color(0, 32, 0));  // LED発光色指定
  strip.setPixelColor(1, strip.Color(0, 0, 0));
  strip.setPixelColor(2, strip.Color(0, 0, 0));
  strip.show();  // 点灯実行

  Serial.begin(115200);

  if (!motor1.begin()) {
    Serial.println("Motor 1 Error");
    while (1);
  }

  motor1.setZeroPoint();
  motor1.setOutput(0);
}

void loop() {

  motor1.update();

  static int noteIndex = 0;
  static unsigned long noteStartTime = 0;
  static bool isFirstRun = true;

  if (isFirstRun) {
    noteStartTime = micros();
    isFirstRun = false;
  }

  // Check if it's time to switch to the next note (using micros for higher precision)
  if (micros() - noteStartTime >= (unsigned long)melody[noteIndex].duration * 1000) {
    noteIndex++;
    if (noteIndex >= sizeof(melody) / sizeof(melody[0])) {
      noteIndex = 0;  // Loop back to start
    }

    noteStartTime = micros();

    // Debug print
    Serial.print("Note: ");
    Serial.print(noteIndex);
    Serial.print(" Freq: ");
    Serial.print(melody[noteIndex].frequency);
    Serial.println("Hz");
  }

  Note currentNote = melody[noteIndex];

  // Sound Generation Logic
  static unsigned long lastToggleMicros = 0;
  static bool outputState = false;
  unsigned long currentMicros = micros();

  if (currentNote.frequency > 0 && currentNote.amplitude > 0) {
    unsigned long halfPeriod = (unsigned long)(1000000.0f / currentNote.frequency / 2.0f);

    if (currentMicros - lastToggleMicros >= halfPeriod) {
      outputState = !outputState;
      lastToggleMicros = currentMicros;
      motor1.setOutput(outputState ? currentNote.amplitude : -currentNote.amplitude);
    }
  } else {
    motor1.setOutput(0);
    outputState = false;  // Reset state during silence
  }
}
