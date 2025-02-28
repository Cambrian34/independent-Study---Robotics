// This code is used to connect the Arduino board to the Lidar sensor and the WiFi module.
// The Arduino board reads the data from the Lidar sensor and sends it to the client via WiFi.


#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

WiFiServer server(8888);
HardwareSerial lidarSerial(1);

void setup() {
    Serial.begin(115200);
    lidarSerial.begin(230400, SERIAL_8N1, 16, 17); // GPIO16, GPIO17

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(1000); }
    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        while (client.connected()) {
            if (lidarSerial.available()) {
                uint8_t data[1024];  
                int len = lidarSerial.readBytes(data, 1024);

                // Send filtered LiDAR data to the Raspberry Pi
                client.write(data, len);
            }
        }
        client.stop();
    }
}