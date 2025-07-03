// Eload First TEST - tests basic functionality 

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Serial Test at: ");
  Serial.println(millis());
  // delay(1);  // delay in between reads for stability
}
