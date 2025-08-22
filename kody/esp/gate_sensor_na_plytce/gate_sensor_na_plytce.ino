#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>

// UART input buffer
String inputString = "";


// MAC Address
uint8_t broadcastAddress[] = {0xE0, 0x98, 0x06, 0x8F, 0xF6, 0xB0}; // Bez
// uint8_t broadcastAddress[] = {0xBC, 0xDD, 0xC2, 0x52, 0x1D, 0xD6}; // Z

uint16_t perc = 1;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // Serial.print("Last Packet Send Status: ");
  // if (sendStatus == 0){
  //   Serial.println("Delivery success");
  // }
  // else{
  //   Serial.println("Delivery fail");
  // }
}

void setup() {
  Serial.begin(115200);  // RX = GPIO3, TX = GPIO1

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  if (esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    // Serial.println("Failed to add peer");
  }
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int val = inputString.toInt();  // Convert directly
      perc = val;

      esp_now_send(broadcastAddress, (uint8_t*)&perc, sizeof(perc));
      inputString = "";  // Reset input
    } else {
      inputString += c;
    }
  }
}

