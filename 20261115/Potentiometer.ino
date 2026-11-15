#define POT_PIN 39

void setup() {
  Serial.begin(115200);
}

void loop() {
  int value = analogRead(POT_PIN);
  Serial.print(value);

  delay(50);
}
