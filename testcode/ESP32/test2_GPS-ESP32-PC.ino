//HardwareSerial Serial2(1);

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  // read from port 2, send to port 0:
  if (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 2:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial2.write(inByte);
  }
}
