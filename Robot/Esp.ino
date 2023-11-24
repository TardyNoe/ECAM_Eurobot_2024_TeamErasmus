#include "WiFi.h"
#include "AsyncUDP.h"
#include <Wire.h>

const char* kSsid = "Raspb";
const char* kPassword = "12345678";
const int kRelayPin = 23;

AsyncUDP udp;

struct RobotData {
  int x = 0;
  int y = 0;
  int angle = 0;
  int distance = 0;
} robotData;

struct ControlData {
  int angleGoal = 0;
  int power = 0;
} controlData;

void receiveEvent(int howMany) {
  int index = 0;
  while (Wire.available() > 1) {
    byte high = Wire.read();
    byte low = Wire.read();
    switch (index) {
      case 0: robotData.x = word(high, low) - 32768; break;
      case 1: robotData.y = word(high, low) - 32768; break;
      case 2: robotData.angle = word(high, low) - 32768; break;
      case 3: robotData.distance = word(high, low) - 32768; break;
    }
    index++;
  }
}

void requestEvent() {
  int dataToSend[2] = {controlData.power, controlData.angleGoal};
  for (int value : dataToSend) {
    Wire.write(highByte(value));
    Wire.write(lowByte(value));
  }
}

void setupWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(kSsid, kPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());
}

void setupUDP() {
  if (udp.listen(1234)) {
    Serial.println("UDP Listening...");
    udp.onPacket([](AsyncUDPPacket packet) {
      byte byteArray[7] = {
        robotData.x & 255, robotData.x >> 8 & 255,
        robotData.y & 255, robotData.y >> 8 & 255,
        robotData.angle & 255, robotData.angle >> 8 & 255,
        robotData.distance & 255
      };
      packet.write(byteArray, sizeof(byteArray));

      uint8_t controlBytes[3];
      size_t lengthToCopy = min(packet.length(), sizeof(controlBytes));
      memcpy(controlBytes, packet.data(), lengthToCopy);
      controlData.angleGoal = (controlBytes[0] << 8) + controlBytes[1] - 180;
      controlData.power = controlBytes[2];
    });
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(kRelayPin, OUTPUT);
  setupWiFi();
  setupUDP();
}

void loop() {
  delay(200); // Consider reducing or removing this delay depending on application
}
