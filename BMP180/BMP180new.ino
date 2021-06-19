#include <Wire.h>
#include <Adafruit_BMP085.h>

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
  
void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
  
void loop() {

    
    double Alt[5];
    double ALT;
    double TBD;//決めた高度

    
    //高度について、5個のデータの移動平均を出す。
     
    //高度のデータを配列に入れる。
    for(int i=0;;i++){ 
    Alt[i] = bmp.readAltitude();
    Altsum = 0;//五個のデータの合計値


    //先に作った配列の中身の和を出して、移動平均を出す。
    for(int k=0 ; k<5 ; k++){
      Altsum = Altsum + Alt[k];
      ALT = Altsum/5
    }

    if(i>5 && ALT<TBD) break;　//高度の移動平均が決定地よりも低かったらループを抜け出す
    //抜け出し方は要検討
    
/*  Serial.print("Tem=,");//tempreture
    Serial.print(bmp.readTemperature());
    Serial.print(",*C ");
    
    Serial.print(",Pre=,");//pressure
    Serial.print(bmp.readPressure());
    Serial.print(",Pa ");
*/   

/* 
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print(",Alt=,");//Altitude
    Serial.print(bmp.readAltitude());
    Serial.print(",meters ");
*/

/*
    Serial.print(",Prs=,");//pressure at sealevel
    Serial.print(bmp.readSealevelPressure());
    Serial.print(",Pa ");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print(",RAl=,");//Real altitude
    Serial.print(bmp.readAltitude(101500));
    Serial.println(",meters");

*/
    if( i==4) i=0;
    //iが4に到達したら、i=0を代入し繰り返す。
    }
    
    Serial.println();
    delay(75);
}
