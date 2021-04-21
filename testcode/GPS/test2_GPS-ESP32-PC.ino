#include <TinyGPS++.h>

TinyGPSPlus gps;
//HardwareSerial Serial2(1);

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
      Serial.print("ALT:  "); Serial.println(gps.altitude.meters());
    }
  }
}
