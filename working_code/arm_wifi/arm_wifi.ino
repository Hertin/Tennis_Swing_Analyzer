#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "send.h"
// IoT platform Credentials

// Internet router credentials
const char* ssid = "hertin";
const char* password = "19921997";

byte num_frame;
float data_buf[MAXNUMFRAME][FRAMELEN];

WiFiServer server(80);
IPAddress IP(192, 168, 4, 1);
IPAddress mask = (255, 255, 255, 0);
WiFiClient client;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(IP, IP, mask);

    delay(500);
    server.begin();
}


void loop() {
    // put your main code here, to run repeatedly:
    IPAddress myAddress = WiFi.softAPIP();

    client = server.available();
    if (client) {
        while (client.connected()) {
            if (client.available()) {
                num_frame = client.read();

                // Read the incoming TCP command
                for (int i = 0; i < num_frame; i++) {
                    while (client.available() < 24);
                    client.read((uint8_t*)data_buf[i], 24);
                }

                send_data(data_buf, num_frame);
            }
        }
    }
}
