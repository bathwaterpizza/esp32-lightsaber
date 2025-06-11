#include <FastLED.h>
#include "OneButton.h"

#define NUM_LEDS 20
#define DATA_PIN 2
#define BTN1_PIN 4
#define BTN2_PIN 15
#define BUZZER_PIN 5

// LED strip
CRGB leds[NUM_LEDS];

// buttons
OneButton btn1;
OneButton btn2;

// time for led color change
unsigned long last_led_time = 0;

void setup() {
  Serial.begin(115200);

  // init LED strip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // breadboard safe

  // init buttons
  btn1.setup(BTN1_PIN, INPUT, false);
  btn2.setup(BTN2_PIN, INPUT, false);
  btn1.attachClick(btn1_pressed);
  btn2.attachClick(btn2_pressed);
}

void loop() {
  unsigned long current_time = millis();

  // check button presses
  btn1.tick();
  btn2.tick();

  // set a random color on each LED every 200ms
  if (current_time - last_led_time > 200) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(random8(), 255, 255);
    }
    FastLED.show();

    last_led_time = current_time;
  }
}

void btn1_pressed() {
  Serial.println("[DEBUG] Button 1 pressed!");
  tone(BUZZER_PIN, 400, 150);
}

void btn2_pressed() {
  Serial.println("[DEBUG] Button 2 pressed!");
  tone(BUZZER_PIN, 800, 150);
}