#include <ESP8266WiFi.h>
#include "receive.h"
const char* ssid     = "hertin";
const char* password = "19921997";

//const char* host = "192.168.137.1";//
IPAddress server(192, 168, 4, 1);
const char* host = "192.168.4.1";

WiFiClient client;
const int port = 80;
int count = 0;

float test_data[20][6] = {{3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}, {3.14, 6.28, 9.42, 12.56, 15.70, 18.84}};

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        // Serial.print(".");
    }
    //Serial.println("wifi connected");
    while (!client.connect(server, port))
    {
        // Serial.println(WiFi.localIP());
        // Serial.println("")
        // Serial.println("connection failed");
        //return;
    }
}

void loop() {
    byte num_frame = 0;
    bool is_successful;
    /*  /WiFi Test

        client.write(20);
        Serial.println();

        Serial.println("start transmit");
        for(int j=0;j<20;j++){
          client.write((byte*)test_data[j],24);

        }
        delay(2000);
        }
        /*/
    if (get_header(&num_frame)) {
        for (frame_index = 0; frame_index < num_frame; frame_index++) {
            is_successful = get_frame(frame_index);
            if (!is_successful) break;
        }
        frame_index = 0;
        //        Serial.println(is_successful ? "finish data receiving" : "cannot receive data");
        if (is_successful) {
            //
            client.write(num_frame);
            for (int k = 0; k < num_frame; k ++) {
                client.write((byte*)sensor_data_frames[k], 24);
            }
        }

    }


}
