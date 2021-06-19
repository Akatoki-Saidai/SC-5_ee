void setup() {  
    Serial.begin(115200);
//    pinMode(PIN_DI1,INPUT_PULLUP);

}

void loop() {
  Serial.println("Hello");
  delay(1000);
//    while(Serial.available()){ //TWELITEからの受信待ち
//        Serial.println(Serial.read());
//    }
}
