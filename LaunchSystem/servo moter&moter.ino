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
unsigned long OnTime = 750;
unsigned long OffTime = 250;
int moterstate = LOW;

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
              for (int pos1 = 0; pos1 <= Angle1; pos1 += 1) {
                  servo1.write(pos1);
                  delay(15);
              }
          Serial.write("Servo moter1 finished rotating \n");
              for (int pos2 = 0; pos2 <= Angle2; pos2 += 1) {
                  servo2.write(pos2);
                  delay(15);
              }
          Serial.write("Servo moter2 finished rotating \n");
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
                        if  ((pos1 < newAngle1) && (currentMillis - previousMillis >= 15)){
                          servo1.write(pos1++);
                          currentMillis = previousMillis;
                        }
                        else if((pos1 > newAngle1) && (currentMillis - previousMillis >= 15)){
                          servo1.write(pos1--);
                          currentMillis = previousMillis;
                        }
                        Serial.write(pos1);
                      }
                    }
                    else{
                      Serial.write("Warming: Out of the range");
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
                  if (Serial.available() ){
                    String key = Serial.readStringUntil(';');
                    Serial.write(key.c_str());
                    Serial.write('\n');
                    int newAngle2 = atoi(key.c_str());
                    if (nowAngle2 != newAngle2){
                      if (newAngle2 <= 180 && newAngle2 >= 0){
                        Serial.write("WARMING: MORER1 IS ROTATING \n");
                        while (pos2 != newAngle2){
                          currentMillis = millis();
                          if ((pos2 < newAngle2) && (currentMillis - previousMillis >= 15)){
                            servo2.write(pos2++);
                            currentMillis = previousMillis;
                          }
                          else if ((pos2 > newAngle2) && (currentMillis - previousMillis >= 15)){
                            servo2.write(pos2--);
                            currentMillis = previousMillis;
                          }
                          Serial.write(pos2);
                        }
                      }
                    else{
                      Serial.write("Warming: Out of the range");
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
