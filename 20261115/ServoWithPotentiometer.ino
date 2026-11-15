#include <ESP32Servo.h>

#define POT_PIN 39
#define SERVO_PIN 14

Servo myServo;


void setup() {
    myServo.attach(SERVO_PIN);
}

void loop() {
    int value = analogRead(POT_PIN);
    myServo.write(value);
    delay(10);

}