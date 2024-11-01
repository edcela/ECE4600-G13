#include <WiFi.h>

// Uses "uofm-guest" WiFi and works, probably in the future it would be better to get it to work on "uofm-secure"
const char* ssid = "uofm-guest";
const char* password = "";

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}

void loop() {
    // Regular loop code
}

