#define TIME_UP 15000   // ms
#define TIME_DOWN 16000 // ms

void setup() {
  Serial.begin(115200);  // RX = GPIO3, TX = GPIO1
  delay(100); // daj STM czas na start

  // Wyślij 2×uint16_t jako bajty (little endian)
  Serial.write(lowByte(TIME_UP));
  Serial.write(highByte(TIME_UP));
  Serial.write(lowByte(TIME_DOWN));
  Serial.write(highByte(TIME_DOWN));
}

void loop() {
  // odbiór percent_closure jako string
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int percent = line.toInt();
    Serial.print("Percent closure: ");
    Serial.print(percent);
    Serial.println("%");
  }
}