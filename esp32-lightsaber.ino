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

// tasks
TaskHandle_t blink_task_handler = nullptr;

void lightsaber_on_off_anim_task(void*) {
  const TickType_t interval = pdMS_TO_TICKS(40);
  const TickType_t holdTime = pdMS_TO_TICKS(200);

  // up
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Red;
    FastLED.show();
    vTaskDelay(interval);
  }

  // wait
  vTaskDelay(holdTime);

  // down
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Black;
    FastLED.show();
    vTaskDelay(interval);
  }

  // end task
  vTaskDelete(nullptr);
}

void lightsaber_blink_anim_task(void*) {
  const TickType_t onTime  = pdMS_TO_TICKS(200);
  const TickType_t offTime = pdMS_TO_TICKS(200);

  while(true) {
    // all on
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
    vTaskDelay(onTime);

    // all off
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    vTaskDelay(offTime);
  }
}

void lightsaber_on_off_sound_task(void*) {
  const uint16_t min_freq = 200;
  const uint16_t max_freq = 1200;
  const TickType_t on_time = pdMS_TO_TICKS(800);
  const TickType_t off_time = pdMS_TO_TICKS(200);
  const uint8_t steps = 20;
  const TickType_t step_delay = on_time / steps;

  // up
  for (uint8_t i = 0; i <= steps; i++) {
    uint16_t freq = min_freq + (uint32_t)(max_freq - min_freq) * i / steps;
    tone(BUZZER_PIN, freq);
    vTaskDelay(step_delay);
  }

  // wait
  noTone(BUZZER_PIN);
  vTaskDelay(off_time);

  // down
  for (uint8_t i = steps; i > 0; i--) {
    uint16_t freq = min_freq + (uint32_t)(max_freq - min_freq) * i / steps;
    tone(BUZZER_PIN, freq);
    vTaskDelay(step_delay);
  }

  // off
  noTone(BUZZER_PIN);
  vTaskDelete(nullptr);
}

void lightsaber_toggle_blink() {
  if (blink_task_handler) {
    vTaskDelete(blink_task_handler);
    blink_task_handler = nullptr;

    // turn off leds
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  } else {
    xTaskCreatePinnedToCore(
      lightsaber_blink_anim_task,
      "BlinkAnimation",
      2048,
      nullptr,
      tskIDLE_PRIORITY + 1,
      &blink_task_handler,
      1
    );
  }
}

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
  /*
  if (current_time - last_led_time > 200) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(random8(), 255, 255);
    }
    FastLED.show();

    last_led_time = current_time;
  }
  */
}

void btn1_pressed() {
  Serial.println("[DEBUG] Button 1 pressed!");
  tone(BUZZER_PIN, 400, 150);

  xTaskCreatePinnedToCore(
    lightsaber_on_off_anim_task,
    "OnOffAnimation",
    2048,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    1
  );
  xTaskCreatePinnedToCore(
    lightsaber_on_off_sound_task,
    "OnOffSound",
    2048,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    1
  );
}

void btn2_pressed() {
  Serial.println("[DEBUG] Button 2 pressed!");

  lightsaber_toggle_blink();
}