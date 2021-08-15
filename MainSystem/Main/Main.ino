int phase = 1;
char key = '0';

int cutparac = 32;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;    //切り離し時の9V電圧を流す時間，単位はsecond

int phase_state = 0;

int launch_PIN = 33;            //トランジスタのピン番号の宣言
int launch_outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond

bool prelaunch = false;
int countdown = 3;
int ignitionstate = false;

int64_t sensorValue_bin[14];
int Datanumber = 0;

//for GPS
#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;

// int i=0;
// int n=0;
// int j=0;
// double gpslat[10];
// double sum_lat;
// double gpslng[10];
// double sum_lng;
double GOAL_lat = 35.862857820;
double GOAL_lng = 139.607681275;
double v_initial= 38.0;  //[m/s]
double g        = 9.80665;  //[m/s/s]
double delta_lng,GPS_lat,GPS_lng,distance,angle_radian,angle_degree;

//for MPU9250
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 21 //I2C通信
#define SCL_MPU 22
#endif

MPU9250_asukiaaa mySensor;
double aX, aY, aZ, aSqrt;


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
int Angle1 = 30;
int Angle2 = 15;
int increment = 1;
unsigned long previousMillis = 0;
unsigned long OnTime = 3000;
unsigned long OffTime = 5000;
int moterstate = LOW;
int interval = 30;
int servophase = 6;
int moter_end = 0;

unsigned long time3, St_Time;


//for BMP180
#include <Wire.h> //I2C通信
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#define SDA_BMP 21
#define SCL_BMP 22

//for SD Card
#include <SPI.h> //SDカードはSPI通信
#include <SD.h>

File CanSatLogData;
File SensorData;

const int sck=13 , miso=15 , mosi=14 , ss=27; 

//センサー値の格納
double Temperature, Pressure, accelX, accelY, accelZ, magX, magY, magZ, gyroX, gyroY, gyroZ, gps_latitude, gps_longitude;
int gps_time;


// Interrupt timer
volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



//サーボモータの角度変更の計算
//int t_caculator(double distance_, v_initial_){
//    theta = atan(power(initial_,2)/(g*distance_) + sqrt(power()))
//}


// Interrupt timer function
void IRAM_ATTR onTimer1(){
    portENTER_CRITICAL_ISR(&timerMux);
    timeCounter1++;
    portEXIT_CRITICAL_ISR(&timerMux);
}


// 64bit整数データのバイナリー変換関数
void casttobyte64(int64_t data, byte buf[]){
    buf[0] = (data >> 56) & 0x00FF;
    buf[1] = (data >> 48) & 0x00FF;
    buf[2] = (data >> 40) & 0x00FF;
    buf[3] = (data >> 32) & 0x00FF;
    buf[4] = (data >> 24) & 0x00FF;
    buf[5] = (data >> 16) & 0x00FF;
    buf[6] = (data >> 8) & 0x00FF;
    buf[7] = (data) & 0x00FF;
}


// 16bit整数データのバイナリー変換関数
void casttobyte16(int16_t data, byte buf[]){
    buf[0] = (data >> 8) & 0x00FF;
    buf[1] = (data) & 0x00FF;
}







void setup() {
    // SD Card initialization
    SPI.begin(sck,miso,mosi,ss);
    SD.begin(ss,SPI);
    CanSatLogData = SD.open("/CanSatLogData.log", FILE_APPEND);
    SensorData = SD.open("/SensorData.bin",FILE_APPEND);

    //無線通信
    Serial2.begin(115200, SERIAL_8N1, 16, 17); //関数内の引数はデータ通信レート，わからん，RXピン，TXピン

    //LED
    pinMode(launch_PIN, OUTPUT);        //点火用トランジスタの出力宣言
    pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
    digitalWrite(launch_PIN, LOW);      //点火用トランジスタの出力オフ
    digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ

    //for MPU9250
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
    // You can set your own offset for mag values
    //Offset値を変える必要あり
    mySensor.magXOffset = -50;
    mySensor.magYOffset = -55;
    mySensor.magZOffset = -10;


    //for GPS
    Serial1.begin(115200, SERIAL_8N1, 5, 18); //関数内の引数はデータ通信レート,わからん,RXピンTXピン


    //for servomoter
    servo1.init(23,0);
    servo2.init(19,1);
    pinMode(4,OUTPUT);
    digitalWrite(4, moterstate);
    Serial2.begin(115200);

    //for BMP
    Wire.begin(SDA_BMP, SCL_BMP);

    int sensorData_d[10];


}//setup関数閉じ


void loop() {
    //割り込み関数（適切にサンプリングレートを確立するために）
    if (timeCounter1 > 0) {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);


        //起動時刻の更新
        unsigned long currentMillis = millis();

        //センサー値のアップデート
        mySensor.accelUpdate();
        mySensor.gyroUpdate();
        mySensor.magUpdate();


        //センサー値取得
        Temperature = bmp.readTemperature();
        Pressure = bmp.readPressure();
        accelX = mySensor.accelX();
        accelY = mySensor.accelY();
        accelZ = mySensor.accelZ();
        magX = mySensor.magX();
        magY = mySensor.magY();
        magZ = mySensor.magZ();
        gyroX = mySensor.gyroX();
        gyroY = mySensor.gyroY();
        gyroZ = mySensor.gyroZ();

        // GPSデータの更新をするかどうか
        if(gps.location.isUpdated()){   //アップデートの実行
            char c = Serial1.read();    //GPSチップからのデータを受信
            gps.encode(c);              //GPSチップからのデータの整形

            gps_latitude = gps.location.lat();
            gps_longitude = gps.location.lng();
            gps_time = gps.time.value();

        }

        
        //地上局からのフェーズ指示
        if(Serial2.available()){

            char key = Serial2.read();

            switch(key){
                case '1':
                    phase = 1;
                    break;

                case '2':
                    phase = 2;
                    break;
                
                case '3':
                    phase = 3;
                    break;

                case '4':
                    phase = 4;
                    break;
                
                case '5':
                    phase = 5;
                    break;
            }
        }//地上局からのフェーズ指示閉じ


    


        //各フェーズごとの記述
        switch (phase){

            //########## 待機フェーズ ##########
            case 1:
            
                if(!phase_state == 1){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase1: transition completed\n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tPhase1: transition completed\n");
                    CanSatLogData.flush();
                    phase_state = 1;
                }
                                
                
                

                break;





            //########## 降下フェーズ ##########
            case 2:
                if(!phase_state == 2){
                    //降下フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase2: transition completed\n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tPhase2: transition completed\n");
                    CanSatLogData.flush();
                    phase_state = 2;
                }




                break;



            //########## 分離フェーズ ##########
            case 3:
                if(!phase_state == 3){
                    //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase3: transition completed\n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tPhase3: transition completed\n");
                    CanSatLogData.flush();
                    time3 = currentMillis; //phase3　開始時間の保存
                    St_Time = time3 + outputcutsecond * 1000;   //基準時間
                    phase_state = 3;
                }

                if(currentMillis > St_Time){
                    digitalWrite(cutparac, LOW); //オフ
                    Serial2.write("WARNING: 9v voltage is stop.\n");
                    if (true){          //条件をうまく適応する方法を今考えてる （加速度の変化が止まったらにしようかなって）
                        phase = 4;
                    }
                }




                break;




            //########## 採取フェーズ ##########
            case 4:
                if(!phase_state == 4){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase4: transition completed\n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tPhase4: transition completed\n");
                    CanSatLogData.flush();
                    phase_state = 4;
                }

                if((moterstate == LOW) && (moter_end == 0) && (currentMillis - previousMillis >= OffTime)){

                    moterstate = HIGH;
                    previousMillis = currentMillis;
                    Serial2.write("Moter start rotating \n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tMotor start rotating\n");
                    CanSatLogData.flush();
                    digitalWrite(4,moterstate);

                }else if((moterstate == HIGH) && (moter_end == 0) && (currentMillis - previousMillis >= OnTime)){
                    moterstate = LOW;
                    previousMillis = currentMillis;
                    Serial2.write("Moter finished rotating \n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tMotor finished rotating\n");
                    CanSatLogData.flush();
                    digitalWrite(4,moterstate);
                    moter_end = 1;
                }
                else if((moterstate == LOW) && (moter_end == 1) && (currentMillis - previousMillis >= OffTime)){
                    previousMillis = currentMillis;
                    phase = 5;
                break;




            //########## 発射フェーズ ##########
            case 5:
                if(!phase_state == 5){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial2.write("Phase5: transition completed\n");
                    CanSatLogData.write(gps_time);
                    CanSatLogData.write("\tPhase5: transition completed\n");
                    CanSatLogData.flush();
                    phase_state = 5;
                }


                switch(servophase){

                    case 6:
                        if(nowAngle1 != Angle1){
                            if((pos1 < Angle1) && (currentMillis - previousMillis >= interval)) {
                                previousMillis = currentMillis;
                                pos1 += increment;
                                servo1.write(pos1);
                                Serial2.write(pos1);
                            }else if ((pos1 > Angle1) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                pos1 -= increment;
                                servo1.write(pos1);
                                Serial2.write(pos1);
                            }
                            nowAngle1 = pos1;
                        }else if(nowAngle1 == Angle1){
                            Serial2.write("******Servo1 finished rotating***** \n");
                            CanSatLogData.write(gps_time);
                            CanSatLogData.write("\tServo1 finished rotating\n");
                            CanSatLogData.flush();
                            servophase = 7;
                            }
                        break;



                    case 7:
                        if(nowAngle2 != Angle2){
                            if((pos2 < Angle2) && (currentMillis - previousMillis >= interval)) {
                                previousMillis = currentMillis;
                                pos2 += increment;
                                servo2.write(pos2);
                                Serial2.write(pos2);
                            }else if ((pos2 > Angle2) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                pos2 -= increment;
                                servo2.write(pos2);
                                Serial2.write(pos2);
                            }
                            nowAngle2 = pos2;
                        }else if(nowAngle2 == Angle2){
                            Serial2.write("******Servo2 finished rotating***** \n");
                            CanSatLogData.write(gps_time);
                            CanSatLogData.write("\tServo2 finished rotating\n");
                            CanSatLogData.flush();
                            servophase = 8;
                        }
                        
                        break;

                break;




        }//フェーズ関数閉じ

        


        //無線通信による指示switch関数
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
                    Serial2.write(pos1);
                }
                else if ((pos1 > Angle1) && (currentMillis - previousMillis >= interval)){
                    previousMillis = millis();
                    servo1.write(pos1--);
                    Serial2.write(pos1);
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
                    Serial2.write(pos2);
                }
                else if ((pos2 > Angle2) && (currentMillis - previousMillis >= interval)){
                    previousMillis = millis();
                    servo2.write(pos2--);
                    Serial2.write(pos2);
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
                                Serial2.write(pos1);
                            }
                            else if((pos1 > newAngle1) && (currentMillis - previousMillis >= interval)){
                                previousMillis = currentMillis;
                                servo1.write(pos1--);
                                Serial2.write(pos1);
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
                                    Serial2.write(pos2);
                                }
                                else if ((pos2 > newAngle2) && (currentMillis - previousMillis >= interval)){
                                    previousMillis = currentMillis;
                                    servo2.write(pos2--);
                                    Serial2.write(pos2);
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


        case 'e':
            prelaunch = false;
            digitalWrite(launch_PIN, LOW);
            ignitionstate = 0;
            countdown = 3;
            Serial2.write("WARNING: The EMERGENCY code has been entered\n");
            key = '0';
            break;

        }//無線通信による指示switch関数閉じ

        Datanumber++;


        //SDカードへデータを保存する
        sensorValue_bin[0] = Temperature * 1000;
        sensorValue_bin[1] = Pressure * 1000;
        sensorValue_bin[2] = accelX * 1000;
        sensorValue_bin[3] = accelY * 1000;
        sensorValue_bin[4] = accelZ * 1000;
        sensorValue_bin[5] = magX * 1000;
        sensorValue_bin[6] = magY * 1000;
        sensorValue_bin[7] = magZ * 1000;
        sensorValue_bin[8] = gyroX * 1000;
        sensorValue_bin[9] = gyroY * 1000;
        sensorValue_bin[10] = gyroZ * 1000;
        sensorValue_bin[11] = gps_latitude * 1000000000;
        sensorValue_bin[12] = gps_longitude * 1000000000;
        sensorValue_bin[13] = gps_time;

        for (int i = 0; i<14; i++) {
                byte buf[8];
                casttobyte64(sensorValue_bin[i],buf);
                SensorData.write(buf,sizeof(buf));
        }



        }
    }//割り込み関数の閉じ
}//loop関数の閉じ