#include <ESP_servo.h>
#include <stdio.h>
#include <string>


ESP_servo servo;
int pos = 0;


void setup() {                //一回だけ実行する
  Serial.begin(115200);
  servo.init(23,0); //pinとチャンネルの選択:
}


void loop() {
  String key;
  if (Serial.available()){
    key = Serial.readStringUntil('\n');
    Serial.println(key);
   //Serial.println("print!");
    int newAngle;
    newAngle = atoi(key.c_str());
    Serial.println(newAngle);
    if (newAngle <= 180 && newAngle >= 0)
    {
      while (pos != newAngle)
      {
        if (pos < newAngle)
        {
          servo.write(pos++);
          delay(15);
        }
        else
        {
          servo.write(pos--);
          delay(15);
        }
        Serial.println(pos);
      }
    }
    else{
      Serial.printf("Out of range [%3d]\n",newAngle);
    }
  }
}
