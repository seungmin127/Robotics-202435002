#define BUZZER_PIN 5

#define BUZZER_FREQ 2000
#define RESOLUTION 8

void setup() {
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, RESOLUTION);

}

void loop() {
  ledcWrite(BUZZER_PIN, 32);
  delay(1000);
  
  ledcWrite(BUZZER_PIN, 64);
  delay(1000);

  ledcWrite(BUZZER_PIN, 128);
  delay(1000);
  
  ledcWrite(BUZZER_PIN, 0);
  delay(1000);

}
