## Description

This project is an ESP32-powered IoT lightsaber that uses motion gestures to control smart home devices via MQTT. By recognizing directional swings using a built-in MPU6050 IMU, the device sends specific commands to a server integrated with Home Assistant. You can trigger actions like toggling lights, playing Spotify tracks, or using TTS for weather updates—all with a flick of the wrist. Users can also record and store custom gesture sequences for new automations. Additional features include onboard LED animations, sound effects via DFPlayer Mini, and real-time debugging through WebSerial.

[Web UI repository](https://github.com/DiogoMarassi/ProjetoMicro)

## Showcase

![schematic](https://github.com/user-attachments/assets/f8e12a7b-529b-471e-b7f1-ea29e5244227)

![assembly1](https://github.com/user-attachments/assets/081de856-7204-4479-b818-d8b53e6aa83b)

https://github.com/user-attachments/assets/079a164e-556b-488e-93ef-55aafcb9b48b

## Dependencies

### Board select

- **ESP32 Wrover Module**

### Arduino Library Manager

- **OneButton** by Matthias
- **FastLED** by Daniel Garcia
- **MPU6050** by Electronic Cats
- **DFPlayer Mini Mp3 by Makuna** by Michael C. Miller
- **ArduinoJson** by Benoit Blanchon
- **WebSerial** by Ayush Sharma

### Import zip library

- [Async TCP](https://github.com/ESP32Async/AsyncTCP)
- [Async MQTT Client](https://github.com/marvinroger/async-mqtt-client)

### Other tools

- [LittleFS Uploader](https://github.com/earlephilhower/arduino-littlefs-upload)
