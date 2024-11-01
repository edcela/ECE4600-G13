#include <esp_now.h>
#include <WiFi.h>

uint8_t baseStationAddress[] = {0xe8, 0x6b, 0xea, 0xdf, 0xe2, 0xdc};  // Replace with Base Station ESP32 MAC

void onSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Updated onReceive function with esp_now_recv_info parameter
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    Serial.print("Received Data from MAC: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(info->src_addr[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" - Data: ");
    for (int i = 0; i < len; i++) {
        Serial.print((char)data[i]);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }

    // Register send and receive callbacks
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, baseStationAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    const char *message = "Data from Drone!";
    esp_now_send(baseStationAddress, (const uint8_t *)message, strlen(message));
    delay(2000);  // Publish data every 2 seconds
}

