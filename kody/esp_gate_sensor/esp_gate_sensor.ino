#include <ESP8266WiFi.h>
#include <espnow.h>

// MAC address of the receiver ESP device
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0x1A, 0x2B, 0x3C}; // example address to be replaced

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed");
    while (true);                 // Halt if init fails
  }

  // Register peer (receiver)
  if (esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) != 0) {
    Serial.println("Failed to add ESP-NOW peer");
    while (true);                 // Halt if peer add fails
  }

  Serial.println("ESP-NOW ready. Waiting for UART input...");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();  // remove \r and whitespace from end

    char msg[250];
    line.toCharArray(msg, sizeof(msg));

    // Send the message via ESP-NOW
    esp_now_send(receiverMAC, (uint8_t *)msg, strlen(msg));

    // debug print
    Serial.print("Sent via ESP-NOW: ");
    Serial.println(line);
  }
}