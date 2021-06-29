#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo;
int lauchc = 33;            //トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond
int pos = 0;
int nowAngle = 0;

void setup(){
    //Serial.begin(115200);
    Serial2.begin(115200);          //無線通信用のデータ転送レート
    pinMode(lauchc, OUTPUT);        //トランジスタの出力宣言
    digitalWrite(lauchc, LOW);      //トランジスタの出力オフ
    servo.init(23,0);
    Serial2.write("TESTING: Serial communication\n");
    Serial2.write("TESTING: Serial communication\n");
}

void loop(){
    if(Serial2.available()){            //無線データに受信があるか
        char key = Serial2.read();      //受信データの1文字を読み込む
        if(key == 'l'){
            Serial2.write("WARNING: The firing code has been entered.\n");
            Serial2.write("WARNING: Are you sure you want to fire it?\n");
            Serial2.write("WARNING: Press the y key to allow firing.\n");

            while(true){

                if(Serial2.available()){
                    char key = Serial2.read();
                    
                    if(key == 'y'){
                        Serial2.write("COUNTDOWN: 3\n");
                        delay(1000);
                        Serial2.write("COUNTDOWN: 2\n");
                        delay(1000);
                        Serial2.write("COUNTDOWN: 1\n");
                        delay(1000);

                        Serial2.write("LAUCHING: 9V voltage is output.\n");
                        digitalWrite(lauchc, HIGH); //オン

                        delay(outputsecond * 1000);

                        Serial2.write("LAUCHING: 9V voltage is stop.\n");
                        digitalWrite(lauchc, LOW); //オフ
                        break;
                    }else if (key == 'n'){
                        Serial2.write("****** Ignition operation canceled ******\n");
                        break;
                    }
                }
            }
        }else if (key == 'e') //緊急停止用
        {
            digitalWrite(lauchc, LOW);
            Serial2.write("WARNING: The EMERGENCY code has been entered\n");
        }else if (key == 'm')
        {
            Serial2.write("****** Motor angle determination mode ******\n");
            Serial2.write("Enter Motor Angle: ");
            while(true){
                if (Serial2.available()){
                    String key = Serial2.readStringUntil('\n');
                    Serial2.write(key);
                    Serial2.write("\n");
                    int newAngle = atoi(key.c_str());
                    if (nowAngle != newAngle){
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
                        Serial2.Write("WARNING: Out of range");
                        }
                    }
                }
            }
        }
    }
}
