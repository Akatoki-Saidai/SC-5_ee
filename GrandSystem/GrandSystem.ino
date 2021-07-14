void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
}

void loop() {
    if(Serial.available()){
        char key = Serial.read();
        Serial2.write(key);
    }

    if(Serial2.available()){
        char receivedtext = Serial2.read();
        Serial.print(receivedtext);
    }
}