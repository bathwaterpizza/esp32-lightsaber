#include <FastLED.h>

#define NUM_LEDS 20
#define DATA_PIN 2


// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
  // init LED strip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // breadboard safe

}

void loop() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(random8(), 255, 255);
  }
  FastLED.show();
  delay(100);

  FastLED.clear();
  FastLED.show();
  delay(100);
}
