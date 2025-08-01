#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// OLED's defines
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 14
#define OLED_SCL 12

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t percentage = 0;

volatile bool newDataReceived = false;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);

void OLED_setup();

void print_percentage(int val);
 
void setup() 
{
  Serial.begin(115200);

  OLED_setup();
  
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) 
  {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
  if (newDataReceived) {
    newDataReceived = false; // Reset the flag
    Serial.println(percentage);
    print_percentage(percentage); // Safe to update display here
  }
}

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&percentage, incomingData, sizeof(percentage));
  newDataReceived = true; // Signal main loop to process
}

void OLED_setup()
{
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 20);
  display.print(" ");
  display.display();
  delay(500);
}

void print_percentage(int val)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Czas: ");
  display.print(val);
  display.println(" ms");
  display.display();
  // delay(500);
}
