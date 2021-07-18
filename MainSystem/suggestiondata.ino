int phase = 0;
int lauchc = 33;            //点火用トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond
int cutparac = 32;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;    //切り離し時の9V電圧を流す時間，単位はsecond
char key = '0';

//for GPS
#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;

int i=0;
int n=0;
int j=0;
float gpslat[10];
float sum_lat;
float gpslng[10];
float sum_lng;  
float GOAL_lat = 35.862857820;
float GOAL_lng = 139.607681275;
float v_initial= 38.0;  //[m/s]
float g        = 9.80665;  //[m/s/s]
float delta_lng,GPS_lat,GPS_lng,distance,angle_radian,angle_degree;

//for MPU9250
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 25
#define SCL_MPU 26
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt;


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
    
    //for GPS
    Serial2.begin(115200, SERIAL_8N1, 5, 18);
  }


void loop() {

    if(Serial2.available()){
        char key = Serial2.read();
        
        //for GPS
        char c = Serial2.read();
        gps.encode(c);
        if (gps.location.isUpdated()) {
           if(n==0){//10個のデータがたまるまで
              gpslat[i] = gps.location.lat();
              gpslng[i] = gps.location.lng();
              i++;
              if(i==10){
                  for(j=0;j<10;j++){
                     sum_lat = sum_lat + gpslat[j];
                     sum_lng = sum_lng + gpslng[j];
                  }
                  GPS_lat=sum_lat/10;
                  GPS_lng=sum_lng/10;
                  delta_lng=GOAL_lng-GPS_lng;
                  distance = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
                  angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
                  angle_degree = angle_radian*360.0/(2.0*M_PI);
                  Serial.print("LAT:  "); Serial.println(GPS_lat, 9);
                  Serial.print("LONG: "); Serial.println(GPS_lng, 9);
                  Serial.print("DISTANCE[m]: "); Serial.println(distance,9);
                  Serial.print("ANGLE[°]: "); Serial.println(angle_degree,9);
                  n++;
                  i=0;
                  sum_lat=0;
                  GPS_lat=0;
                  sum_lng=0;
                  GPS_lng=0;
              }
            }
            else{//10個のデータがたまった後
                  for(j=0;j<9;j++){
                     gpslat[j]=gpslat[j+1]; 
                     gpslng[j]=gpslng[j+1];
                  }
                  gpslat[9]=gps.location.lat();
                  gpslng[9]=gps.location.lng();
                  for(j=0;j<10;j++){
                     sum_lat = sum_lat + gpslat[j];
                     sum_lng = sum_lng + gpslng[j];
                  }
                  GPS_lat=sum_lat/10;
                  GPS_lng=sum_lng/10;
                  delta_lng=GOAL_lng-GPS_lng;
                  distance = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
                  angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
                  angle_degree = angle_radian*360.0/(2.0*M_PI);
                  Serial.print("LAT:  "); Serial.println(GPS_lat, 9);
                  Serial.print("LONG: "); Serial.println(GPS_lng, 9);
                  Serial.print("DISTANCE[m]: "); Serial.println(distance,9);
                  Serial.print("ANGLE[°]: "); Serial.println(angle_degree,9);
                  sum_lat=0;
                  GPS_lat=0;
                  sum_lng=0;
                  GPS_lng=0;
           }
        }
        
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

            case 3: //分離フェーズ
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
