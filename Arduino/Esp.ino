#include "WiFi.h"
#include "AsyncUDP.h"
#include <Wire.h>
const char* ssid = "Livebox-F0E62";
const char* pass = "mW69KnaMhLatv9UQhM";
const int rele = 23;
AsyncUDP udp;
int datax = 0;
int datay = 0;
int dataa = 0;
int datadist = 0;

int angleGoal = -180;
int power = 0;

void requestEvent() {
  Wire.begin(8);
  Wire.write(angleGoal);  
  Wire.write(power);
}

void setup() {
    Serial.begin(115200);
    Wire.onRequest(requestEvent);
    pinMode(rele, OUTPUT);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    if (udp.listen(1234)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            //uint8_t byteArray[] = {0x01, 0x02, 0x03, 0x04};
            byte byteArray[7];        
            byteArray[0] = datax & 255;
            byteArray[1] = (datax >> 8)  & 255;
            byteArray[2] = datay & 255;
            byteArray[3] = (datay >> 8)  & 255;
            byteArray[4] = dataa & 255;
            byteArray[5] = (dataa >> 8)  & 255;
            byteArray[6] = datadist & 255;
            size_t byteArraySize = sizeof(byteArray) / sizeof(byteArray[0]);
            packet.write(byteArray, byteArraySize);

            uint8_t byteArray2[3];
            size_t lengthToCopy = packet.length() > sizeof(byteArray2) ? sizeof(byteArray2) : packet.length();
            memcpy(byteArray2, packet.data(), lengthToCopy);
            angleGoal = (byteArray2[0] << 8) + byteArray2[1] -180;
            power = byteArray2[2];
        });
    }
}

void loop() {
    delay(100);
    Wire.requestFrom(9, 2); 
    byte x1,x2;
    x1 = Wire.read();  
    x2 = Wire.read();
    datax = (int)x1 << 8 | (int)x2;
    Serial.println(datax);
    x1 = Wire.read();
    x2 = Wire.read();
    datay = (int)x1 << 8 | (int)x2;
    x1 = Wire.read();
    x2 = Wire.read();
    dataa = (int)x1 << 8 | (int)x2;
    x1 = Wire.read();
    datadist = (int)x1;
  }
