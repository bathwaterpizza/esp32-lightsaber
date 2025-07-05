#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <FastLED.h>
#include <DFMiniMp3.h>
#include <OneButton.h>

// debug options
#define ENABLE_NETWORKING 0

// LED strip
#define NUM_LEDS 20
#define LED_DATA_PIN 18
CRGB leds[NUM_LEDS];

// buttons
#define BTN1_PIN 4
#define BTN2_PIN 15
OneButton btn1;
OneButton btn2;

// Wi-Fi credentials
const char* WIFI_SSID     = "Projeto";
const char* WIFI_PASSWORD = "2022-11-07";

// MQTT
AsyncMqttClient mqttClient;
const char* MQTT_HOST          = "192.168.0.100";
const uint16_t MQTT_PORT       = 1883;
const char* MQTT_CLIENT_ID     = "ESP32_Lightsaber";
const char* MQTT_LOGIN         = "mqttuser";
const char* MQTT_PASSWORD      = "1234";
const char* MQTT_TOPIC_GESTURE = "esp32/saber/gesture";

// DFPlayer Mini
#define DFPLAYER_RX_PIN 26
#define DFPLAYER_TX_PIN 27
HardwareSerial mp3Serial(2);
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3; 
DfMp3 dfmp3(mp3Serial);

// tasks
TaskHandle_t blink_task_handler = nullptr;

// MPU6050 IMU
#define IMU_INT_PIN 25
#define IMU_SCL_PIN 22
#define IMU_SDA_PIN 21

// DFPlayer Mini event handlers
class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action)
  {
    if (source & DfMp3_PlaySources_Sd) 
    {
        Serial.print(F("[DEBUG] DFPlayer SD Card, "));
    }
    if (source & DfMp3_PlaySources_Usb) 
    {
        Serial.print(F("[DEBUG] DFPlayer USB Disk, "));
    }
    if (source & DfMp3_PlaySources_Flash) 
    {
        Serial.print(F("[DEBUG] DFPlayer Flash, "));
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode)
  {
    Serial.println();
    Serial.print(F("[DEBUG] DFPlayer Com Error "));
    Serial.println(errorCode);
  }
  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track)
  {
    Serial.print(F("[DEBUG] DFPlayer finished playing #"));
    Serial.println(track);
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};

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
      0
    );
  }
}

// wifi events
void on_wifi_connected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println(F("[DEBUG] WiFi connected"));

  mqttClient.connect();
}
void on_wifi_disconnected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println(F("[DEBUG] WiFi disconnected, retrying.."));

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// mqtt events
void on_mqtt_connected(bool sessionPresent) {
  Serial.println(F("[DEBUG] MQTT connected"));
}
void on_mqtt_disconnected(AsyncMqttClientDisconnectReason reason) {
  Serial.print(F("[DEBUG] MQTT disconnected, retrying.."));

  mqttClient.connect();
}

// max evt 64 chars!
void publish_event(const char* topic, const char* evt) {
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"event\":\"%s\",\"ts\":%lu}", evt, millis());
  mqttClient.publish(topic, 1, false, buf);
}

void setup() {
  Serial.begin(115200);

  // init LED strip
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  //FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // breadboard safe

  // init buttons
  btn1.setup(BTN1_PIN, INPUT, false);
  btn2.setup(BTN2_PIN, INPUT, false);
  btn1.attachClick(btn1_pressed);
  btn2.attachClick(btn2_pressed);

  // init DFPlayer Mini
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  dfmp3.begin(); // RX, TX
  dfmp3.reset();
  delay(300);
  dfmp3.setVolume(30);
  // test: play track 1
  dfmp3.playMp3FolderTrack(1); // sd:/mp3/0001.mp3

#if ENABLE_NETWORKING
  // init MQTT
  mqttClient.onConnect(on_mqtt_connected);
  mqttClient.onDisconnect(on_mqtt_disconnected);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_LOGIN, MQTT_PASSWORD);
  mqttClient.setClientId(MQTT_CLIENT_ID);

  // init Wi-Fi
  WiFi.onEvent(on_wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(on_wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  // connect to wifi, which will then connect to mqtt
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif
}

void loop() {
  unsigned long current_time = millis();

  // check button events
  btn1.tick();
  btn2.tick();

  // check DFPlayer Mini events
  dfmp3.loop();
}

void btn1_pressed() {
  Serial.println(F("[DEBUG] Button 1 pressed!"));

  xTaskCreatePinnedToCore(
    lightsaber_on_off_anim_task,
    "OnOffAnimation",
    2048,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    0
  );

  // test example
  //publish_event(MQTT_TOPIC_GESTURE, "gesture-kitchen-lights");
}

void btn2_pressed() {
  Serial.println(F("[DEBUG] Button 2 pressed!"));

  lightsaber_toggle_blink();
}