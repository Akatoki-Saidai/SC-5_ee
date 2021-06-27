#include <ESP_servo.h>

ESP_servo servo1; //サーボのオブジェクト宣言:

int lauchc = 33;            //トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond

void setup(){
    //Serial.begin(115200);
    Serial2.begin(115200);          //無線通信用のデータ転送レート
    pinMode(lauchc, OUTPUT);        //トランジスタの出力宣言
    digitalWrite(lauchc, LOW);      //トランジスタの出力オフ
    Serial2.write("TESTING: Serial communication\n");
    Serial2.write("TESTING: Serial communication\n");

    servo1.init(23,0); //pinとチャンネルの選択:
    servo1.write(0); //初めの角度:
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
                    }

                    else if (key == 'a'){  
                        Serial2.write("LAUNCH ANGLE. \n");
                         for (int pos = 0; pos <= 45; pos += 1) {            //打ち上げ角度を決める
                           servo1.write(pos); 
                           delay(30);             //時間はどうしよっか？
                        break;
                    }
                }
                    
                    else if (key == 'n'){
                        Serial2.write("****** Ignition operation canceled ******\n");
                        break;
                    }
                    
            }
        }
        }
        
        else if (key == 'e') //緊急停止用
        {
            digitalWrite(lauchc, LOW);
            Serial2.write("WARNING: The EMERGENCY code has been entered\n");
        }
    }
}
