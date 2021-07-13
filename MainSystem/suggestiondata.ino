int phase = 0;
int lauchc = 33;            //点火用トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond
int cutparac = 32;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;    //切り離し時の9V電圧を流す時間，単位はsecond
char key = '0';

//for MPU9250
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 25
#define SCL_MPU 26
#endif
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt;

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
    Wire.begin(SDA_PIN, SCL_PIN);
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
    
    //for BMP180
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
  }


void loop() {

    if(Serial2.available()){
        char key = Serial2.read();

        switch (key)
        {
        case '1':
            phase = 1;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '2':
            phase = 2;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '3':
            phase = 3;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '4':
            phase = 4;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '5':
            phase = 5;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;

        default:
            break;
        }
    }


    switch (phase)
        {
            case 1: //待機フェーズ
                Serial2.Write("Phase1: transition completed\n");
                Serial2.Write("");
                
                //フェーズ1  MPU9250使用  機体の傾きを測定
                Serial.println("You are in the phase 1");


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
                    Serial.println("Cannod read accel values");
                }
                phase = 2;

            case 2: //降下フェーズ

                //フェーズ2  BMP180使用  加速度の移動平均を測定
                Serial.println("You are in the phase 2");
                double Alt[5];
                double ALT;
                double TBD;//決めた高度
  
                //高度について、5個のデータの移動平均を出す。
                for(int i=0;;i++){     //高度のデータを配列に入れる。
                    Alt[i] = bmp.readAltitude();
  
                    double Altsum = 0;   //五個のデータの合計値
  
                    //先に作った配列の中身の和を出して、移動平均を出す
                    for(int k=i-5 ; k==i ; k++){
                        Altsum = Altsum + Alt[k];
                        ALT = Altsum/5
                    }
                    if(i>5 && ALT<TBD) break;　//高度の移動平均が決定地よりも低かったらループを抜け出す 
                }
    
                Serial.println();

            case 3: //分離フェーズ
                Serial.println("You are in the phase 3");
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

            case 4: //採取フェーズ
                Serial.println("You are in the phase 4");
                
            case 5: //発射フェーズ
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


char readcommand(){
    // 受信データがあった時だけ、処理を行う
    if (Serial2.available()) {       // 受信データがあるか？
        key = Serial2.read();            // 1文字だけ読み込む
        switch (key)
        {
            case '1':
                phase = 1;
                Serial2.write("****** Phase transition command accepted ******\n");
                break;
            case '2':
                phase = 2;
                Serial2.write("****** Phase transition command accepted ******\n");
                break;
            case '3':
                phase = 3;
                Serial2.write("****** Phase transition command accepted ******\n");
                break;
            case '4':
                phase = 4;
                Serial2.write("****** Phase transition command accepted ******\n");
                break;
            case '5':
                phase = 5;
                Serial2.write("****** Phase transition command accepted ******\n");
                break;

            default:
                return key;
        }
}

void caculator(){
    
}
