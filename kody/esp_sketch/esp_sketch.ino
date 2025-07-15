void setup() {
  Serial.begin(115200);  // RX = GPIO3, TX = GPIO1
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();  // delete \r and whitespace
    Serial.print("Gate position: ");
    Serial.println(line);
  }
}