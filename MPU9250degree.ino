#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

void setup() {
  Serial.begin(115200);
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
  mySensor.magXOffset = 86;
  mySensor.magYOffset = -15;
  mySensor.magZOffset = -43;
}

void loop() {
  uint8_t sensorId;

  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();

    double tang = mX/mY;  //tanθの値
    double deg = atan(tang); //角度の仮の値
    double degree; //実際の角度がここに入る

    //値をそのまま使うと-90~90degの範囲でしか表示されない
    // mXの値を使って0~360表示に直したい
    if ( tang > 0 ){  
      if ( mX > 0 ){
        degree = deg;
      }else if( mX < 0 ){
        degree = deg + 180;
      }
    }
    
    else if ( tang < 0 ){
      if ( mX > 0 ){
        degree = deg + 180;
      }else if( mX < 0 ){
        degree = deg;
      }
    }
    
  Serial.print("degree = ");
  Serial.println(String(degree));//角度表示
  delay(500);
  }

}
