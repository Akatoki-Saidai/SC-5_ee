#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo1;
ESP_servo servo2;
int pos1 = 0;
int pos2 = 0;
int nowAngle1 = 0;
int nowAngle2 = 0;
int Angle1 = 120;
int Angle2 = 45;

unsigned long previousMillis = 0;
unsigned long OnTime = 7500;
unsigned long OffTime = 250;
int moterstate = LOW;
int interval = 30;

char key = '0';


void setup() {
  servo1.init(23,0);
  servo2.init(19,1);
  pinMode(4,OUTPUT);
  digitalWrite(4, moterstate);
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis(); 

  
  if(Serial.available()){
      char key = Serial.read();
  
  
  switch(key){
      
          case 'i':
            while(true){
              currentMillis = millis();
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
                break;
             }
          }
          previousMillis = currentMillis;
          key = '0';
          break;

      
          case 'j':
          if(nowAngle1 != Angle1){
          while(pos1 != Angle1){
            currentMillis = millis();
            if((pos1 < Angle1) && (currentMillis - previousMillis >= interval)) {
              previousMillis = millis();
              servo1.write(pos1++);
              Serial.println(pos1);
            }
            else if ((pos1 > Angle1) && (currentMillis - previousMillis >= interval)){
              previousMillis = millis();
              servo1.write(pos1--);
              Serial.println(pos1);
            }
          }
          }
          Serial.write("******Servo1 finished rotating***** \n");
          if(nowAngle2 != Angle2){
          while(pos2 != Angle2){
            currentMillis = millis();
            if((pos2 < Angle2) && (currentMillis - previousMillis >= interval)) {
              previousMillis = millis();
              servo2.write(pos2++);
              Serial.println(pos2);
            }
            else if ((pos2 > Angle2) && (currentMillis - previousMillis >= interval)){
              previousMillis = millis();
              servo2.write(pos2--);
              Serial.println(pos2);
            }
          }
          }
          Serial.write("******Servo2 finished rotating*****\n");
          nowAngle1 = Angle1;
          nowAngle2 = Angle2;
          key = '0';
          break;
     
          case 'm':
              Serial.write("****** Servo Motor1 plung angle determination mode ******\n");
              Serial.write("Enter Motor Angle: ");
              while(true){
                if (Serial.available() ){
                  String key = Serial.readStringUntil(';');
                  Serial.write(key.c_str());
                  Serial.write('\n');
                  int newAngle1 = atoi(key.c_str());
                  if (nowAngle1 != newAngle1){
                    if (newAngle1 <= 180 && newAngle1 >= 0){
                      Serial.write("WARMING: MORER1 IS ROTATING \n");
                      while (pos1 != newAngle1){
                        currentMillis = millis();
                        if  ((pos1 < newAngle1) && (currentMillis - previousMillis >= interval)){
                          previousMillis = currentMillis;
                          servo1.write(pos1++);
                          Serial.println(pos1);
                        }
                        else if((pos1 > newAngle1) && (currentMillis - previousMillis >= interval)){
                          previousMillis = currentMillis;
                          servo1.write(pos1--);
                          Serial.println(pos1);
                        }
                      }
                    }
                    else{
                      Serial.write("Warming: Out of the range \n");
                      }
                    nowAngle1 = newAngle1;
                  }
                  break;
               }
             }
             currentMillis = previousMillis;
             key = '0';
             break;
       
             case 'n':
                Serial.write("****** Servo Motor2 launch angle determination mode ******\n");
                Serial.write("Enter Motor Angle: ");
                while(true){
                  if (Serial.available()){
                    String key = Serial.readStringUntil(';');
                    Serial.write(key.c_str());
                    Serial.write('\n');
                    int newAngle2 = atoi(key.c_str());
                    if (nowAngle2 != newAngle2){
                      if (newAngle2 <= 180 && newAngle2 >= 0){
                        Serial.write("WARMING: MORER1 IS ROTATING \n");
                        while (pos2 != newAngle2){
                          currentMillis = millis();
                          if ((pos2 < newAngle2) && (currentMillis - previousMillis >= interval)){
                            previousMillis = currentMillis;
                            servo2.write(pos2++);
                            Serial.println(pos2);
                          }
                          else if ((pos2 > newAngle2) && (currentMillis - previousMillis >= interval)){
                            previousMillis = currentMillis;
                            servo2.write(pos2--);
                            Serial.println(pos2);
                          }
                        }
                      }
                    else{
                      Serial.write("Warming: Out of the range\n");
                    }
                    nowAngle2 = newAngle2;
                  }
                  break;
                }
              }
              currentMillis = previousMillis;
              key = '0';
              break;
      }
       
  }
}
