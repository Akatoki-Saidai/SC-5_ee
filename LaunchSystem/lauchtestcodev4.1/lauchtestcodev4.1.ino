#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo1;
ESP_servo servo2;

int pos1 = 0;
int pos2 = 0;

int nowAngle1 = 0;
int nowAngle2 = 0;

unsigned long previousMillis = 0;
unsigned long OnTime = 7500;
unsigned long OffTime = 250;
int moterstate = LOW;
int interval = 30;

int launch_PIN = 33;            //トランジスタのピン番号の宣言
int launch_outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond

bool prelaunch = false;
int countdown = 3;
int ignitionstate = false;
char key = '0';

void setup(){
    Serial.begin(115200);          //無線通信用のデータ転送レート
    pinMode(launch_PIN, OUTPUT);        //トランジスタの出力宣言
    digitalWrite(launch_PIN, LOW);      //トランジスタの出力オフ
    servo1.init(23,0);
    servo2.init(19,1);
    pinMode(4,OUTPUT);
    digitalWrite(4, moterstate);
    Serial.setTimeout(10000);
    Serial.write("TESTING: Serial communication\n");
    Serial.write("TESTING: Serial communication\n");
}

void loop(){
    
    //Number of milliseconds passed since the program started
    unsigned long currentMillis = millis(); 




    //Confirmation of received data instructions
    if(Serial.available()){

        //Read one character of the receive key
        key = Serial.read();
    }




    //Correspondence according to the receiving key
    switch(key){

        case 'l':
            Serial.write("WARNING: The firing code has been entered.\n");
            Serial.write("WARNING: Are you sure you want to fire it?\n");
            Serial.write("WARNING: Press the y key to allow firing.\n");
            prelaunch = true;
            key = '0';
            break;

        // case 'm':
        //     prelaunch = false;
        //     Serial.write("****** Motor angle determination mode ******\n");
        //     Serial.write("Enter Motor Angle: ");

        //     while(true){
        //         if(Serial.available()){
        //             String rotation_angle = Serial.readStringUntil(';');
        //             Serial.write(rotation_angle.c_str());
        //             Serial.write("\n");
        //             int newAngle = atoi(rotation_angle.c_str());
        //             if (nowAngle != newAngle){
        //                 Serial.write("WARNING: MOTOR IS ROTATING\n");
        //                 if (newAngle <= 180 && newAngle >= 0){
        //                     while (pos != newAngle){
        //                         if (pos < newAngle){
        //                             servo.write(pos++);
        //                             delay(15);
        //                         }else{
        //                             servo.write(pos--);
        //                             delay(15);
        //                         }
        //                             Serial.write(pos);
        //                     }
        //                 }else{
        //                     Serial.write("WARNING: Out of range");
        //                 }
        //                 nowAngle = newAngle;
        //             }
        //             break;
        //         }

        //     }
            
        //     key = '0';
        //     break;   


        case 'm':
            Serial.write("****** Servo Motor1 plung angle determination mode ******\n");
            Serial.write("Enter Motor Angle: ");
            while(true){
                if (Serial.available()){
                    String key = Serial.readStringUntil(';');
                    Serial.write(key.c_str());
                    Serial.write('\n');
                    int newAngle1 = atoi(key.c_str());
                    if (nowAngle1 != newAngle1){
                        if(newAngle1 <= 180 && newAngle1 >= 0){
                            Serial.write("WARMING: MORER1 IS ROTATING \n");
                            while (pos1 != newAngle1){
                                currentMillis = millis();
                                if((pos1 < newAngle1) && (currentMillis - previousMillis >= interval)){
                                    previousMillis = currentMillis;
                                    servo1.write(pos1++);
                                    Serial.println(pos1);
                                }else if((pos1 > newAngle1) && (currentMillis - previousMillis >= interval)){
                                    previousMillis = currentMillis;
                                    servo1.write(pos1--);
                                    Serial.println(pos1);
                                }
                            }
                        }else{
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
                          }else if((pos2 > newAngle2) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                servo2.write(pos2--);
                                Serial.println(pos2);
                          }
                        }
                      }else{
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


        case 'y':
            if(prelaunch){
                if(ignitionstate){
                    if(currentMillis - previousMillis >= launch_outputsecond * 1000){
                        Serial.write("LAUCHING: 9V voltage is stop.\n");
                        digitalWrite(launch_PIN, LOW); //オフ
                        ignitionstate = 0;
                        countdown = 3;
                        prelaunch = false;
                        key = '0';
                    }
                }else if(currentMillis - previousMillis >= 1000){
                    char c_countdown = '0' + countdown;
                    Serial.write("COUNTDOWN: ");
                    Serial.write(c_countdown);
                    Serial.write("\n");
                    --countdown;
                    if(countdown+1<=0){
                        Serial.write("LAUCHING: 9V voltage is output.\n");
                        digitalWrite(launch_PIN, HIGH); //オン
                        ignitionstate = true;
                    }
                    previousMillis = currentMillis;
                }
            }
            break;
        
        case 'e':
            prelaunch = false;
            digitalWrite(launch_PIN, LOW);
            ignitionstate = 0;
            countdown = 3;
            Serial.write("WARNING: The EMERGENCY code has been entered\n");
            key = '0';
            break;

        default:
            break;
    }
}
