#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo1;
ESP_servo servo2;
int pos1 = 0;
int pos2 = 0;
int nowAngle1 = 0;
int nowAngle2 = 0;
int Angle1=30;
int Angle2=15;
int increment = 1;
unsigned long previousMillis = 0;
unsigned long OnTime = 30000;
unsigned long OffTime = 250;
int moterstate = LOW;
int interval = 30;

int phase = 1;
int servophase = 5;
char key = '0';

int phase_state = 0;


void setup() {
  //for servomoter
    servo1.init(23,0);
    servo2.init(19,1);
    pinMode(4,OUTPUT);
    digitalWrite(4, moterstate);
    Serial.begin(115200);

}

void loop() {
  //起動時刻の更新
    unsigned long currentMillis = millis();
  
  if(Serial.available()){
        char key = Serial.read();
        switch (key){
          case '1':
            phase = 1;
            break;
           case '2':
            phase = 2;
            break;
        }
        }
    

    switch (phase){
        case 1:
          if(!phase_state == 1){
            //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase4: transition completed\n");
            Serial.write("");
            phase_state = 1;
          }
                
          if((moterstate == LOW) && (currentMillis - previousMillis >= OffTime)){
            moterstate = HIGH;
            previousMillis = currentMillis;
            Serial.write("Moter start rotating \n");
            digitalWrite(4,moterstate);
          }
          else if((moterstate == HIGH) && (currentMillis - previousMillis >= OnTime)){
            moterstate = LOW;
            previousMillis = currentMillis;
            Serial.write("Moter finished rotating \n");
            digitalWrite(4,moterstate);
            phase = 2;
          }
          break;

              
         case 2:
           if(!phase_state == 2){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase5: transition completed\n");
                    Serial2.write("");
                    phase_state = 2;
                }
           if(Serial.available()){
            char key = Serial.read();
            switch (key){
              case '5':
                phase = 5;
                break;
              case '6':
                phase = 6;
                break;
            }
          }
          switch(servophase){
           case 5:
           if(nowAngle1 != Angle1){
                    if((pos1 < Angle1) && (currentMillis - previousMillis >= interval)) {
                        previousMillis = currentMillis;
                        pos1 += increment;
                        servo1.write(pos1);
                        Serial.println(pos1);
                    }
                    else if ((pos1 > Angle1) && (currentMillis - previousMillis >= interval)){
                        previousMillis = currentMillis;
                        pos1 -= increment;
                        servo1.write(pos1);
                        Serial.println(pos1);
                    }
                }
                else if(nowAngle1 == Angle1){
                  Serial.write("******Servo1 finished rotating***** \n");
                  servophase = 6;
                }
                break;

            case 6:
            if(nowAngle2 != Angle2){
                    if((pos2 < Angle2) && (currentMillis - previousMillis >= interval)) {
                        previousMillis = currentMillis;
                        pos2 += increment;
                        servo1.write(pos2);
                        Serial.println(pos2);
                    }
                    else if ((pos2 > Angle2) && (currentMillis - previousMillis >= interval)){
                        previousMillis = currentMillis;
                        pos2 -= increment;
                        servo1.write(pos2);
                        Serial.println(pos2);
                    }
                }
                else if(nowAngle2 == Angle2){
                  Serial.write("******Servo2 finished rotating***** \n");
                }
                break;
          }
          break;
    }
           
}
