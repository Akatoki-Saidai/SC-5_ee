
#include "ESP_servo.h"

ESP_servo servo;

void setup() {
  servo.init(23,0);
}



void loop() {
  for (int pos = 10; pos <= 170; pos += 1) {
    servo.write(pos);
    delay(15);
  }
  for (int pos = 170; pos >= 10; pos -= 1) {
    servo.write(pos);
    delay(15);
  }
}

