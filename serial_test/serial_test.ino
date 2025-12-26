void setup() {
  // Initialize Serial Communication at 115200 bits per second
  Serial.begin(115200); 
}

void loop() {
  Serial.println("Hello from ESP32!"); // Send text to computer
  delay(1000);                         // Wait 1 second
}
