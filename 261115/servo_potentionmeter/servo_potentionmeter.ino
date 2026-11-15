#include <ESP32Servo.h>

#define SERVO_PIN 14
#define POT_PIN 39

Servo myServo;

void setup() {
    Serial.begin(115200);
    myServo.attach(SERVO_PIN);
    myServo.write(0);
}

void loop() {
    int value = analogRead(POT_PIN);
    Serial.println(value);
    int speed = map(value,0,4095,0,180);
    myServo.write(speed);
}