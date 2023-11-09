#include <Arduino.h>
#include <WiFi.h>
#include <BluetoothSerial.h>

const char *ssid = "Balise";
const char *password = "ERASMUSTEAM";
const char *serverIP = "192.168.137.1";
const int serverPort = 8080;

const long CHECK_INTERVAL = 5000;
unsigned long lastCheckTime = 0;
bool wasBTConnected = false;
bool wasWiFiConnected = false;

BluetoothSerial SerialBT;
WiFiClient client;

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  SerialBT.begin("Coccinelle1");
}

void checkConnection()
{
    unsigned long currentTime = millis();
    if(currentTime - lastCheckTime >= CHECK_INTERVAL)
    {
        bool isBTConnected = SerialBT.connected();
        if (isBTConnected != wasBTConnected)
        {
            wasBTConnected = isBTConnected;

            Serial.printf("Bluetooth %s\n", isBTConnected ? "connecté" : "déconnecté");
        }

        bool isWiFiConnected = client.connected();
        if (isWiFiConnected != wasWiFiConnected)
        {
            wasWiFiConnected = isWiFiConnected;

            Serial.printf("WiFi %s\n", isWiFiConnected ? "connecté" : "déconnecté");
        }

        lastCheckTime = currentTime;
    }
}

void loop()
{
    if (SerialBT.available())
    {
        char c = SerialBT.read();
        Serial.write(c);
    }

    if (Serial.available())
    {
        char c = Serial.read();
        SerialBT.write(c);
        client.write(c);
    }

    if (client.available())
    {
        char c = client.read();
        Serial.write(c);
    }

    checkConnection();
}
