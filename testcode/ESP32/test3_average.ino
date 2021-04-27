#include <TinyGPS++.h>

TinyGPSPlus gps;
//HardwareSerial Serial2(1);
int i=0;
void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  double lati[10];
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("Start(i): "); Serial.println(i);
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
      lati[i] = gps.location.lat()+ 10*i;
      Serial.println(lati[i],9);
      i++;
      if(i==10){//i=10のときに配列表示
        Serial.print("\n");
        for(int j=0;j<10;j++){//配列表示
         Serial.println(lati[j],9);
        }
        Serial.print("\n");
        i=0;
      }
    }
  }
}
