int phase = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  }

void loop() {
    switch (phase)
        {
            case 0: //待機フェーズ
                Serial2.println("Phase0: transition completed");
                Serial2.println("");
            
            case 1: 

            case 2:

            case 3:

            case 4:

            default:
                switch (phase)
                {
                    case 6:
                        phase = 1;
                        Serial2.println("Phase1: Phase transition completed") ;
                    case 7:
                        phase = 2;
                        Serial2.println("Phase2: Phase transition completed") ;
                    case 8:
                        phase = 3;
                        Serial2.println("Phase3: Phase transition completed") ;
                    case 9:
                        phase = 4;
                        Serial2.println("Phase4: Phase transition completed") ;
                }

        }
}


void readcommand(){
    // 受信データがあった時だけ、処理を行う
    if (Serial2.available()) {       // 受信データがあるか？
        char key = Serial2.read();            // 1文字だけ読み込む
        Serial2.write(key); // 1文字送信。受信データをそのまま送り返す。
        Serial.print(key);
        phase = key ;
}

void caculator(){
    
}