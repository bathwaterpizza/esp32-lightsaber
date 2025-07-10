#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <FastLED.h>
#include <DFMiniMp3.h>
#include <OneButton.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <ArduinoJson.h>
#include <vector>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

// debug web server
AsyncWebServer webServer(80);

// --- debug options ---
#define ENABLE_NETWORKING 1
#define ENABLE_LEDS 1
#define ENABLE_DFPLAYER 1
#define ENABLE_BUTTONS 1
#define ENABLE_IMU 1

// --- LED strip ---
#define NUM_LEDS 19
#define LED_DATA_PIN 23
CRGB leds[NUM_LEDS];

// --- buttons ---
#define BTN1_PIN 4
#define BTN2_PIN 15
OneButton btn1;
OneButton btn2;

// --- WiFi credentials ---
const char* WIFI_SSID     = "Projeto";
const char* WIFI_PASSWORD = "2022-11-07";

// --- MQTT ---
AsyncMqttClient mqttClient;
const char* MQTT_HOST      = "192.168.0.136";
const uint16_t MQTT_PORT   = 1883;
const char* MQTT_CLIENT_ID = "ESP32_Lightsaber";
const char* MQTT_LOGIN     = "mqttuser";
const char* MQTT_PASSWORD  = "1234";
const char* MQTT_TOPIC_POST_NEW_GESTURE     = "sabre/comando";
const char* MQTT_TOPIC_GET_GESTURES         = "sabre/comando/gesto";
const char* MQTT_TOPIC_POST_GESTURE_SUCCESS = "sabre/comando/feito";

// --- DFPlayer Mini ---
#define DFPLAYER_RX_PIN 26
#define DFPLAYER_TX_PIN 27
HardwareSerial mp3Serial(2);
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
DfMp3 dfmp3(mp3Serial);

// --- MPU6050 IMU + DMP Config ---
#define IMU_INT_PIN 25
#define IMU_SCL_PIN 22
#define IMU_SDA_PIN 21
MPU6050 mpu;
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

// DMP output structures
Quaternion q;
VectorInt16 aa, aaReal, aaWorld;
VectorFloat gravity;

// Gesture detection parameters
const int16_t GESTURE_THRESHOLD = 10000;  // ~0.5g threshold (tune as needed)
const int GESTURE_DEBOUNCE_MS = 400; // ignore motion interrupts for 300ms after detection
unsigned long lastGestureTime = 0;

// --- gesture ---
const char* GESTURE_FORWARD  = "F";
const char* GESTURE_BACKWARD = "T";
const char* GESTURE_UP       = "C";
const char* GESTURE_DOWN     = "B";
const char* GESTURE_LEFT     = "E";
const char* GESTURE_RIGHT    = "D";
bool recordingNewGesture = false;
bool checkingGesture = false;
std::vector<String> gestureBuffer;
// struct to pass color param to tasks
struct StrobeParams {
  CRGB swingColor;
};
TaskHandle_t currentLedTask = nullptr;

// DFPlayer Mini event handlers class
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

// helper to save gestures json to flash
bool save_gestures_to_flash(const uint8_t* payload, size_t len) {
  StaticJsonDocument<256> probe;
  DeserializationError err = deserializeJson(probe, payload, len);
  if (err) {
    Serial.print(F("[DEBUG] JSON parse error: ")); Serial.println(err.c_str());
    return false;
  }

  File f = LittleFS.open("/data.json", "w");
  if (!f) {
    Serial.println(F("[DEBUG] LittleFS open failed"));
    return false;
  }

  f.write(payload, len);
  f.close();

  Serial.println(F("[DEBUG] New gesture list saved to /data.json"));
  return true;
}

// color lookup
CRGB color_from_name(const char* name) {
  if      (!strcasecmp(name, "vermelho")) return CRGB::Red;
  else if (!strcasecmp(name, "verde"))    return CRGB::Green;
  else if (!strcasecmp(name, "amarelo"))  return CRGB::Yellow;
  else if (!strcasecmp(name, "branco"))   return CRGB::White;
  else if (!strcasecmp(name, "roxo"))     return CRGB::Purple;
  else if (!strcasecmp(name, "azul"))     return CRGB::Blue;
  else if (!strcasecmp(name, "laranja"))  return CRGB::Orange;
  else                                    return CRGB::Blue;
}

// --- freertos tasks ---
void lightsaber_on_anim_task(void*) {
  const TickType_t interval = pdMS_TO_TICKS(40);

  // wait 1s for DFPlayer "lightsaber on" sound
  vTaskDelay(pdMS_TO_TICKS(1000));

  // slowly turn leds on, bottom to top
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    vTaskDelay(interval);
  }

  // end task
  vTaskDelete(nullptr);
}

void lightsaber_swing_strobe_task(void* pv) {
  // retrieve color param
  StrobeParams* p = static_cast<StrobeParams*>(pv);
  const CRGB swingCol = p->swingColor;
  delete p;

  const TickType_t on  = pdMS_TO_TICKS(50);
  const TickType_t off = pdMS_TO_TICKS(50);

  for (uint8_t i = 0; i < 5; ++i) {
    fill_solid(leds, NUM_LEDS, swingCol);
    FastLED.show();
    vTaskDelay(on);

    FastLED.clear(true);
    vTaskDelay(off);
  }

  // return to idle blue
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();

  currentLedTask = nullptr;
  vTaskDelete(nullptr);
}

// --- WiFi events ---
void on_wifi_connected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println(F("[DEBUG] WiFi connected"));

  mqttClient.connect();

  // print the IP so you know where to point your browser:
  Serial.print  (F("[DEBUG] IP address: "));
  Serial.println(WiFi.localIP());

  // start WebSerial now that we have an IP
  webServer.begin();
  WebSerial.begin(&webServer);
  Serial.print  (F("[DEBUG] WebSerial at http://"));
  Serial.print  (WiFi.localIP());
  Serial.println(F("/webserial"));
}
void on_wifi_disconnected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println(F("[DEBUG] WiFi disconnected, retrying.."));

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// --- MQTT events ---
void on_mqtt_connected(bool sessionPresent) {
  Serial.println(F("[DEBUG] MQTT connected"));

  // subscribe to receive gesture list updates
  mqttClient.subscribe(MQTT_TOPIC_GET_GESTURES, 1);
}
void on_mqtt_disconnected(AsyncMqttClientDisconnectReason reason) {
  Serial.print(F("[DEBUG] MQTT disconnected, retrying.."));

  mqttClient.connect();
}
void on_mqtt_data(char* topic, char* payload,
                  AsyncMqttClientMessageProperties,
                  size_t len, size_t, size_t) {
  // check if it's the gesture list update topic
  if (strcmp(topic, MQTT_TOPIC_GET_GESTURES) != 0) return;

  Serial.print(F("[DEBUG] MQTT received gesture list update. Payload len="));
  Serial.println(len);
  save_gestures_to_flash((uint8_t*)payload, len);
}


// debug function: read /data.json from LittleFS and print to Serial
void debug_print_stored_gestures()
{
  if (!LittleFS.exists("/data.json")) {
    Serial.println(F("[DEBUG] /data.json not found"));
    return;
  }

  File f = LittleFS.open("/data.json", "r");
  if (!f) {
    Serial.println(F("[DEBUG] Failed to open /data.json"));
    return;
  }

  // Use a sufficiently large document; 4 KB fits ~1200-byte JSON comfortably
  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.print  (F("[DEBUG] JSON parse error: "));
    Serial.println(err.c_str());
    return;
  }

  Serial.println(F("----- Stored gesture library -----"));
  serializeJsonPretty(doc, Serial);          // nicely indented output
  Serial.println();                          // final newline
  Serial.println(F("-----------------------------------"));
}

// --- OneButton events ---
void btn1_pressed() {
  Serial.println(F("[DEBUG] Button 1 pressed!"));

  recordingNewGesture = true;
  gestureBuffer.clear();
}

void btn2_pressed() {
  Serial.println(F("[DEBUG] Button 2 pressed!"));

  checkingGesture = true;
  gestureBuffer.clear();
}

void btn1_released() {
  Serial.println(F("[DEBUG] Button 1 released!"));

  recordingNewGesture = false;

  // ignore if no gestures
  if (gestureBuffer.empty()) return;

  // build payload JSON
  StaticJsonDocument<256> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const String& g : gestureBuffer) arr.add(g);
  String payload;
  serializeJson(arr, payload);

  Serial.print(F("[DEBUG] Sending gesture list: "));
  Serial.println(payload);

#if ENABLE_NETWORKING
  mqttClient.publish(MQTT_TOPIC_POST_NEW_GESTURE,
                    1, // QoS 1
                    false,
                    payload.c_str(),
                    payload.length());
#endif
}

void btn2_released() {
  Serial.println(F("[DEBUG] Button 2 released!"));

  checkingGesture = false;

  if (gestureBuffer.empty()) { // nothing recorded
    strobe_color("vermelho");
    return;
  }

  // load stored gestures
  File f = LittleFS.open("/data.json", "r");
  if (!f) { Serial.println(F("[DEBUG] /data.json open failed")); strobe_color("vermelho"); return; }

  StaticJsonDocument<4096> doc;
  if (deserializeJson(doc, f)) { f.close(); strobe_color("vermelho"); return; }
  f.close();

  // search for matching gesture list
  const char* matchedColor = nullptr;
  JsonArray*  matchedGesto = nullptr;

  for (JsonObject obj : doc.as<JsonArray>()) {
    JsonArray gestoArr = obj["gesto"];
    if (gestoArr.size() != gestureBuffer.size()) continue;

    bool same = true;
    for (size_t i = 0; i < gestoArr.size(); ++i) {
      if (gestureBuffer[i] != (const char*)gestoArr[i]) { same = false; break; }
    }

    if (same) {
      matchedColor = obj["cor"];
      matchedGesto = &gestoArr;
      break;
    }
  }

  // check match
  if (matchedGesto) {
    Serial.println(F("[DEBUG] Gesture matched!"));

    // LED feedback
    strobe_color(matchedColor);

#if ENABLE_NETWORKING
    // publish the matched gesture list itself
    StaticJsonDocument<256> out;
    JsonArray outArr = out.to<JsonArray>();
    for (JsonVariant v : *matchedGesto) outArr.add(v);

    char buf[256];
    size_t n = serializeJson(outArr, buf);

    mqttClient.publish(MQTT_TOPIC_POST_GESTURE_SUCCESS, 1, false, buf, n);
#endif
  } else {
    Serial.println(F("[DEBUG] No match - red strobe"));
    strobe_color("vermelho");
  }
}

void setup() {
  Serial.begin(115200);

#if ENABLE_NETWORKING
  // --- init LittleFS ---
  if (!LittleFS.begin()) {
    Serial.println(F("[DEBUG] LittleFS mount failed, formatting…"));
    LittleFS.format();
    LittleFS.begin();
  }
#endif

#if ENABLE_LEDS
  // --- init LED strip ---
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  //FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // breadboard safe
#endif

#if ENABLE_BUTTONS
  // --- init buttons ---
  btn1.setup(BTN1_PIN, INPUT, false);
  btn2.setup(BTN2_PIN, INPUT, false);
  btn1.setPressMs(0);
  btn2.setPressMs(0);
  btn1.attachLongPressStart(btn1_pressed);
  btn2.attachLongPressStart(btn2_pressed);
  btn1.attachLongPressStop(btn1_released);
  btn2.attachLongPressStop(btn2_released);
#endif

#if ENABLE_DFPLAYER
  // --- init DFPlayer Mini ---
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  dfmp3.begin(); // RX, TX
  dfmp3.reset();
  delay(300);
  dfmp3.setVolume(30);

  // play lightsaber "on" sound and animation
  dfmp3.playMp3FolderTrack(1);
#endif
#if ENABLE_LEDS
  xTaskCreatePinnedToCore(
    lightsaber_on_anim_task,
    "LightsaberOnAnimation",
    4096,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    0
  );
#endif

#if ENABLE_IMU
  // --- init MPU6050 IMU + DMP ---
  Wire.begin();                       
  mpu.initialize();
  
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // offset calibration
    mpu.setXAccelOffset( 898);
    mpu.setYAccelOffset(-2296);
    mpu.setZAccelOffset( 1118);
    mpu.setXGyroOffset ( 167);
    mpu.setYGyroOffset ( -52);
    mpu.setZGyroOffset ( -55);

    mpu.setDMPEnabled(true);
    pinMode(IMU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(F("[DEBUG] MPU6050 DMP initialized."));
  } else {
    Serial.print(F("[DEBUG] DMP init failed (code ")); Serial.print(devStatus); Serial.println(F(")"));
  }
#endif

#if ENABLE_NETWORKING
  // --- init MQTT ---
  mqttClient.onConnect(on_mqtt_connected);
  mqttClient.onDisconnect(on_mqtt_disconnected);
  mqttClient.onMessage(on_mqtt_data);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_LOGIN, MQTT_PASSWORD);
  mqttClient.setClientId(MQTT_CLIENT_ID);

  // --- init WiFi ---
  WiFi.onEvent(on_wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(on_wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  // connect to wifi, which will then connect to mqtt
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif
}

void strobe_color(const char* color) {
  // stop current animation
  if (currentLedTask != nullptr) {
    vTaskDelete(currentLedTask); // kill last task
    currentLedTask = nullptr;
    FastLED.clear(true);
  }

  // allocate color param
  StrobeParams* param = new StrobeParams{ color_from_name(color) };

  xTaskCreatePinnedToCore(
    lightsaber_swing_strobe_task,          // task function
    "SwingStrobe",            // name
    4096,                     // stack words
    param,                    // parameter
    tskIDLE_PRIORITY + 1,     // priority
    &currentLedTask,
    0                         // pin to core 0
  );
}

void loop() {
#if ENABLE_BUTTONS
  // check button events
  btn1.tick();
  btn2.tick();
#endif

#if ENABLE_DFPLAYER
  // check DFPlayer Mini events
  dfmp3.loop();
#endif

#if ENABLE_IMU
  // check IMU interrupt
  if (!dmpReady) return;  // skip IMU reading if DMP failed to init
  if (mpuInterrupt) {
    mpuInterrupt = false;
    // Read FIFO packet
    uint16_t fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
      Serial.println(F("[DEBUG] MPU6050 FIFO Overflow!"));
      mpu.resetFIFO();  // FIFO overflow, reset
      return;
    }
    if (fifoCount < packetSize) return;  // not a full packet yet (shouldn't happen with interrupt)
    // Read all available packets, we only need the last
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // Get DMP data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    // Check for gesture
    int16_t ax = aaWorld.x;
    int16_t ay = aaWorld.y;
    int16_t az = aaWorld.z;
    unsigned long now = millis();

    if (now - lastGestureTime > GESTURE_DEBOUNCE_MS) {  // gesture cooldown time
      if (abs(ax) > GESTURE_THRESHOLD || abs(ay) > GESTURE_THRESHOLD || abs(az) > GESTURE_THRESHOLD) {
#if ENABLE_DFPLAYER
        // play random lightsaber swing sound effect
        if (recordingNewGesture || checkingGesture) dfmp3.playMp3FolderTrack(random(2, 6));
#endif

#if ENABLE_LEDS
        if (recordingNewGesture || checkingGesture) strobe_color("azul");
#endif

        String gesture;
        if (abs(ax) >= abs(ay) && abs(ax) >= abs(az)) {
          gesture = (ax > 0 ? GESTURE_FORWARD : GESTURE_BACKWARD);
        } else if (abs(ay) >= abs(ax) && abs(ay) >= abs(az)) {
          gesture = (ay > 0 ? GESTURE_RIGHT : GESTURE_LEFT);
        } else {
          gesture = (az > 0 ? GESTURE_UP : GESTURE_DOWN);
        }

        Serial.println(String("Gesture detected: ") + gesture);
        WebSerial.println(String("Gesture detected: ") + gesture);
        lastGestureTime = now;

        if (recordingNewGesture || checkingGesture) {
          gestureBuffer.push_back(gesture);
        }
      }
    }
  }
#endif
}
