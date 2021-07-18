int phase = 1;
int lauchc = 33;            //点火用トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond
int cutparac = 32;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;    //切り離し時の9V電圧を流す時間，単位はsecond
char key = '0';
int phase_state = 0;

//for MPU9250
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 25
#define SCL_MPU 26
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt;


//for servomoter
#include <ESP_servo.h>
#include <stdio.h>
#include <string>

ESP_servo servo1;
ESP_servo servo2;
int pos1 = 0;
int pos2 = 0;
int nowAngle1 = 0;
int nowAngle2 = 0;
int Angle1;
int Angle2;
unsigned long previousMillis = 0;
unsigned long OnTime = 30000;
unsigned long OffTime = 250;
int moterstate = LOW;
int interval = 30;



//for BMP180
#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#define SDA_BMP 21
#define SCL_BMP 22


void setup() {
    Serial2.begin(115200);
    pinMode(lauchc, OUTPUT);        //点火用トランジスタの出力宣言
    pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
    digitalWrite(lauchc, LOW);      //点火用トランジスタの出力オフ
    digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ

    //for MPU9250
    while(!Serial);
    Serial.println("started");
    #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    //Wire.begin(SDA_MPU, SCL_MPU);
    mySensor.setWire(&Wire);
    #endif
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
    // You can set your own offset for mag values
    //Offset値を変える必要あり
    mySensor.magXOffset = -50;
    mySensor.magYOffset = -55;
    mySensor.magZOffset = -10;

    //for servomoter
    servo1.init(23,0);
    servo2.init(19,1);
    pinMode(4,OUTPUT);
    digitalWrite(4, moterstate);
    Serial2.begin(115200);
  }


void loop() {
    unsigned long currentMillis = millis();
    altitude = bmp.readAltitude();



    if(Serial2.available()){
        char key = Serial2.read();
        if((key = '1') || (key == '2') || (key == '3') || (key == '4') || (key == '5')){
            phase = key;
        }
    }

    switch (phase)
        {





            //########## 待機フェーズ ##########
            case 1: 

                if(!phase_state == 1){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase1: transition completed\n");
                    Serial2.Write("");
                    Wire.begin(SDA_MPU, SCL_MPU);
                    phase_state = 1;
                }

                double TBD;       //加速度TBD以上でphase2に移行
                uint8_t sensorId;
                if (mySensor.readId(&sensorId) == 0) {
                    Serial.println("sensorId: " + String(sensorId));
                } else {
                    Serial.println("Cannot read sensorId");
                }
                while (mySensor.accelUpdate() == 0) {
                aSqrt = mySensor.accelSqrt();

                if(aSqrt>TBD) break;
                } else {
                    Serial2.Write("Cannod read accel values");
                }
                phase = 2;





            //########## 降下フェーズ ##########
            case 2:
                if(!phase_state == 2){
                    //降下フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.Write("Phase2: transition completed\n");
                    Serial2.Write("");
                    phase_state = 2;
                }

                //フェーズ2  BMP180使用  加速度の移動平均を測定
                Wire.begin(SDA_BMP, SCL_BMP);
                Serial2.Write("You are in the phase 2");
                double Alt[];
                double Altsum = 0;   //五個のデータの合計値
                double ALT;          //五個のデータの平均値
                double TBD_h;        //高度TBD

                //高度について、5個のデータの移動平均を出す。
                while(i>5 && ALT<TBD_h){   //高度の移動平均が決定地よりも低かったらループを抜け出す
                    //先に作った配列の中身の和を出して、移動平均を出す
                    for(int k=i-5 ; k==i ; k++){
                        Altsum = Altsum + Alt[k];
                        ALT = Altsum/5
                    }


                Serial.println();






            //########## 分離フェーズ ##########
            case 3:
                if(!phase_state == 3){
                    //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.Write("Phase3: transition completed\n");
                    Serial2.Write("");
                    phase_state = 3;
                }

                Serial2.write("WARNING: The cut-para code has been entered.\n");
                digitalWrite(cutparac, HIGH); //オン
                Serial2.write("WARNING: 9v voltage is output.\n");
                delay(outputcutsecond * 1000);
                digitalWrite(cutparac, LOW); //オフ
                Serial2.write("WARNING: 9v voltage is stop.\n");
                Serial2.write("Phase3: Process all completed. Enter '4' key.\n");

                while(true){
                    if(Serial2.available()){
                        key = Serial2.read();
                        if (key == '4'){
                            break;
                        }
                    }
                }




            //########## 採取フェーズ ##########
            case 4:
                if(!phase_state == 4){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.Write("Phase4: transition completed\n");
                    Serial2.Write("");
                    phase_state = 4;
                }
                
                }

                phase = 5;
                Serial2.write("You are in the phase 4");

            case 5: //発射フェーズ
                if(!phase_state == 5){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.Write("Phase1: transition completed\n");
                    Serial2.Write("");
                    phase_state = 5;
                }
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
                        }
                    }







    }



    switch(key){

        case 'i':
        while(true){
            currentMillis = millis();
            if((moterstate == LOW) && (currentMillis - previousMillis >= OffTime)){
                moterstate = HIGH;
                previousMillis = currentMillis;
                Serial2.write("Moter start rotating \n");
                digitalWrite(4,moterstate);
            }
            else if((moterstate == HIGH) && (currentMillis - previousMillis >= OnTime)){
                moterstate = LOW;
                previousMillis = currentMillis;
                Serial2.write("Moter finished rotating \n");
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
                Serial2.println(pos1);
            }
            else if ((pos1 > Angle1) && (currentMillis - previousMillis >= interval)){
                previousMillis = millis();
                servo1.write(pos1--);
                Serial2.println(pos1);
            }
        }
    }
    Serial2.write("******Servo1 finished rotating***** \n");
    if(nowAngle2 != Angle2){
        while(pos2 != Angle2){
            currentMillis = millis();
            if((pos2 < Angle2) && (currentMillis - previousMillis >= interval)) {
                previousMillis = millis();
                servo2.write(pos2++);
                Serial2.println(pos2);
            }
            else if ((pos2 > Angle2) && (currentMillis - previousMillis >= interval)){
                previousMillis = millis();
                servo2.write(pos2--);
                Serial2.println(pos2);
            }
        }
    }
    Serial2.write("******Servo2 finished rotating*****\n");
    nowAngle1 = Angle1;
    nowAngle2 = Angle2;
    key = '0';
    break;

    case 'm':
    Serial2.write("****** Servo Motor1 plung angle determination mode ******\n");
    Serial2.write("Enter Motor Angle: ");
    while(true){
        if (Serial2.available() ){
            String key = Serial2.readStringUntil(';');
            Serial2.write(key.c_str());
            Serial2.write('\n');
            int newAngle1 = atoi(key.c_str());
            if (nowAngle1 != newAngle1){
                if (newAngle1 <= 180 && newAngle1 >= 0){
                    Serial2.write("WARMING: MORER1 IS ROTATING \n");
                    while (pos1 != newAngle1){
                        currentMillis = millis();
                        if  ((pos1 < newAngle1) && (currentMillis - previousMillis >= interval)){
                            previousMillis = currentMillis;
                            servo1.write(pos1++);
                            Serial2.println(pos1);
                        }
                        else if((pos1 > newAngle1) && (currentMillis - previousMillis >= interval)){
                            previousMillis = currentMillis;
                            servo1.write(pos1--);
                            Serial2.println(pos1);
                        }
                    }
                }
                else{
                    Serial2.write("Warming: Out of the range \n");
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
    Serial2.write("****** Servo Motor2 launch angle determination mode ******\n");
    Serial2.write("Enter Motor Angle: ");
    while(true){
        if (Serial2.available()){
            String key = Serial2.readStringUntil(';');
            Serial2.write(key.c_str());
            Serial2.write('\n');
            int newAngle2 = atoi(key.c_str());
            if (nowAngle2 != newAngle2){
                if (newAngle2 <= 180 && newAngle2 >= 0){
                    Serial2.write("WARMING: MORER1 IS ROTATING \n");
                        while (pos2 != newAngle2){
                            currentMillis = millis();
                            if ((pos2 < newAngle2) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                servo2.write(pos2++);
                                Serial2.println(pos2);
                            }
                            else if ((pos2 > newAngle2) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                servo2.write(pos2--);
                                Serial2.println(pos2);
                            }
                        }
                    }
                    else{
                        Serial2.write("Warming: Out of the range\n");
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
