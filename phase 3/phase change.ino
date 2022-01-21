#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 21
#define SCL_MPU 22
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt;

int type_state = 1;
int type = 1;
float outputcutsecond = 4.0;
float accelsqrt,accel;
float time1,time2,currentMillis,StTime;

//type2でのみ利用する変数
int i,j;
float Accel[100000];           //計測した値を全ておいておく関数
float Preac,differ,Acsum,Acave,RealDiffer;

void setup() {

    Serial.begin(115200);
    while(!Serial);
    Serial.println("started");
    #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_MPU, SCL_MPU);
    mySensor.setWire(&Wire);
    #endif
    
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
    //mySensor.accelSqrt();
    // You can set your own offset for mag values
    //Offset値を変える必要あり
    mySensor.magXOffset = -50;
    mySensor.magYOffset = -55;
    mySensor.magZOffset = -10;

  //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
  Serial.print("Phase3: transition completed\n");
  Serial.println("");
  time1 = millis();                          //phase3に入った時の時間
  float St_Time = time1 + outputcutsecond * 1000;        //基準時間
}

void loop() {

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    accelsqrt = mySensor.accelSqrt();
    currentMillis = millis();
  
    switch(type){

      case 1:

      if(!type_state == 1){
        //電流フェーズに入ったとき１回だけ実行したいプログラムを書く
        Serial.println("type1: transition completed\n");
        //Serial2.Write("");
        type_state = 2;

        Serial.println("WARNING: The cut-para code has been entered.\n");
        //digitalWrite(cutparac, HIGH); //オン
        time2 = currentMillis;
        Serial.println("WARNING: 9v voltage is output.\n");
      }

 
      if(time2 > StTime){
        //digitalWrite(cutparac, LOW); //オフ
        Serial.println("WARNING: 9v voltage is stop.\n");
        //phase = 4;
        type = 2;
      }


      case 2:

      if(!type_state == 2){
        //停止フェーズに入ったとき１回だけ実行したいプログラムを書く
        Serial.println("type2: transition completed\n");
        //Serial2.Write("");
        type_state = 3;
        i = 0;
        j = 0;
        Preac = 0;      //1秒前の加速度を記憶
        differ = 0.1;   //移動平均の差
      }

    accel = accelsqrt;
    Acsum = 0;         //加速度5個の合計値
    Acave = 0;         //加速度5個の平均値
    RealDiffer = 0;       //1秒前との差を記憶する

   if (i < 5){          
     Accel[i] = accel;
     i = i + 1;
   }else{          //データが五個集まったとき
     Accel[i] = accel;
     for(j=i-4 ; j==i ; j++){
       Acsum = Acsum + Accel[j];
        i = i + 1;
     }
     Acave = Acsum / 5;
     RealDiffer = Preac - Acave;
      if( RealDiffer < differ ){ //移動平均が基準以内の変化量だったた時
       Serial.println("YOU CAN MOVE THE NEXT PHASE");
       type = 3;
      //phase = 4;
      }
      Preac = Acave;    //次のループでは今のデータと比較する
    }
  }
  }
}
