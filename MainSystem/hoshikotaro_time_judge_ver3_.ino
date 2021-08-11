            //for BMP180(suggestiondata.ino 68行目)
            int phase=1;    

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
}

void loop(){
            //########## 待機フェーズ ########## (suggestiondata.ino 227行目)
          
          switch(phase)
          {
            case 1:
            Serial.write("YOU ARE IN THE PHASE(case1:n==0): "); Serial.println(phase);
            delay(10000);//10秒松
            phase = 2;
            
            //########## 降下フェーズ ##########
            case 2:
            Serial.write("YOU ARE IN THE PHASE(case2): "); Serial.println(phase);
            delay(2000);//2秒まつ
            phase = 3;
            Serial.write("YOU ARE IN THE PHASE(case2): "); Serial.println(phase);
          }
}
