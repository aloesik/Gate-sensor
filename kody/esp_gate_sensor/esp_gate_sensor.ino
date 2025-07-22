#define TIME_UP 15000
#define TIME_DOWN 16000

void setup() {
  Serial.begin(115200);
  /*sendTime(TIME_UP);
  sendTime(TIME_DOWN);*/
}

void loop()
{
    Serial.write(0xA5);  // testowy bajt
    delay(200);
}

void sendTime(uint16_t value) {
  Serial.write((uint8_t*)&value, 2);
}