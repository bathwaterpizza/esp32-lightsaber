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

// --- debug options ---
#define ENABLE_NETWORKING 0

// --- LED strip ---
#define NUM_LEDS 20
#define LED_DATA_PIN 18
CRGB leds[NUM_LEDS];

// --- buttons ---
#define BTN1_PIN 4
#define BTN2_PIN 15
OneButton btn1;
OneButton btn2;

// --- WiFi credentials ---
const char* WIFI_SSID     = "Peres";
const char* WIFI_PASSWORD = "pan13DKs48";

// --- MQTT ---
AsyncMqttClient mqttClient;
const char* MQTT_HOST      = "broker.mqtt.cool";
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
const int16_t GESTURE_THRESHOLD = 8000;  // ~0.5g threshold (tune as needed)
const int GESTURE_DEBOUNCE_MS = 300; // ignore motion interrupts for 300ms after detection
unsigned long lastGestureTime = 0;

// --- gesture ---
const char* GESTURE_FORWARD  = "forward";
const char* GESTURE_BACKWARD = "backward";
const char* GESTURE_UP       = "up";
const char* GESTURE_DOWN     = "down";
const char* GESTURE_LEFT     = "left";
const char* GESTURE_RIGHT    = "right";
bool recordingNewGesture = false;
std::vector<String> gestureBuffer;

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

void lightsaber_on_anim_task(void*) {
  const TickType_t interval = pdMS_TO_TICKS(40);

  // slowly turn leds on, bottom to top
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    vTaskDelay(interval);
  }

  // end task
  vTaskDelete(nullptr);
}

// --- WiFi events ---
void on_wifi_connected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println(F("[DEBUG] WiFi connected"));

  mqttClient.connect();
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

void setup() {
  Serial.begin(115200);

  // --- init LittleFS ---
  if (!LittleFS.begin()) {
    Serial.println(F("[DEBUG] LittleFS mount failed, formatting…"));
    LittleFS.format();
    LittleFS.begin();
  }

  // --- init LED strip ---
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  //FastLED.setMaxPowerInVoltsAndMilliamps(5, 500); // breadboard safe

  // --- init buttons ---
  btn1.setup(BTN1_PIN, INPUT, false);
  btn2.setup(BTN2_PIN, INPUT, false);
  btn1.setPressMs(0);
  btn2.setPressMs(0);
  btn1.attachLongPressStart(btn1_pressed);
  btn2.attachLongPressStart(btn2_pressed);
  btn1.attachLongPressStop(btn1_released);
  btn2.attachLongPressStop(btn2_released);

  // --- init DFPlayer Mini ---
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  dfmp3.begin(); // RX, TX
  dfmp3.reset();
  delay(300);
  dfmp3.setVolume(30);

  // play lightsaber "on" sound and animation
  dfmp3.playMp3FolderTrack(1);
  xTaskCreatePinnedToCore(
    lightsaber_on_anim_task,
    "LightsaberOnAnimation",
    2048,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    0
  );

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

void loop() {
  // check button events
  btn1.tick();
  btn2.tick();

  // check DFPlayer Mini events
  dfmp3.loop();

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
        // play random lightsaber swing sound effect
        if (recordingNewGesture) dfmp3.playMp3FolderTrack(random(2, 6));

        String gesture;
        if (abs(ax) >= abs(ay) && abs(ax) >= abs(az)) {
          gesture = (ax > 0 ? GESTURE_FORWARD : GESTURE_BACKWARD);
        } else if (abs(ay) >= abs(ax) && abs(ay) >= abs(az)) {
          gesture = (ay > 0 ? GESTURE_RIGHT : GESTURE_LEFT);
        } else {
          gesture = (az > 0 ? GESTURE_UP : GESTURE_DOWN);
        }

        Serial.println(String("Gesture detected: ") + gesture);
        lastGestureTime = now;

        if (recordingNewGesture) {
          gestureBuffer.push_back(gesture);
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
//  Debug helper: read /data.json from LittleFS and pretty-print to Serial
// -----------------------------------------------------------------------------
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

  // debug
  debug_print_stored_gestures();
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
}
