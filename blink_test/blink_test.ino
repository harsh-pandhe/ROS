void setup() {
  pinMode(2, OUTPUT);
}

void loop() {
  digitalWrite(2, HIGH);  // Turn LED on
  delay(500);             // Wait
  digitalWrite(2, LOW);   // Turn LED off
  delay(500);             // Wait
}
