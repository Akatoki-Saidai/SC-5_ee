#include "ESP_servo.h"
#include "Arduino.h"

ESP_servo::ESP_servo()
{
}
void ESP_servo::init(uint8_t pin, uint8_t channel)
{
  CHANNEL = channel;

  //PWM freqency [Hz]
  uint16_t freqency = 50;
  servo_0 = 0.0005f * freqency * 1024;
  servo_180 = 0.0024f * freqency * 1024;

  uint8_t bit = 10;

  ledcSetup(CHANNEL, freqency, bit);

  ledcAttachPin(pin, CHANNEL);
}

void ESP_servo::write(uint16_t degree)
{
  int value = map(degree, 0, 180, servo_0, servo_180);
  int Normalized_value = constrain(value, min, max);
  //ledcWrite(channel, value);
  ledcWrite(CHANNEL, value);
}
