int phase = 0;
int lauchc = 33;            //点火用トランジスタのピン番号の宣言
int outputsecond = 5;       //点火時の9V電圧を流す時間，単位はsecond
int cutparac = 32;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;    //切り離し時の9V電圧を流す時間，単位はsecond
char key = '0';


void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    pinMode(lauchc, OUTPUT);        //点火用トランジスタの出力宣言
    pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
    digitalWrite(lauchc, LOW);      //点火用トランジスタの出力オフ
    digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
  }


void loop() {

    if(Serial2.available()){
        char key = Serial2.read();

        switch (key)
        {
        case '1':
            phase = 1;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '2':
            phase = 2;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '3':
            phase = 3;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '4':
            phase = 4;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;
        case '5':
            phase = 5;
            Serial2.write("****** Phase transition command accepted ******\n");
            break;

        default:
            break;
        }
    }


    switch (phase)
        {
            case 1: //待機フェーズ
                Serial2.Write("Phase1: transition completed\n");
                Serial2.Write("");

            case 2: //降下フェーズ

            case 3: //分離フェーズ
                Serial2.write("WARNING: The cut-para code has been entered.\n");
                digitalWrite(cutparac, HIGH); //オン
                Serial2.write("WARNING: 9v voltage is output.\n");
                delay(outputcutsecond * 1000);
                digitalWrite(cutparac, LOW); //オフ
                Serial2.write("WARNING: 9v voltage is stop.\n");
                Serial2.write("Phase3: Process all completed. Enter '4' key.\n");

                while(true){
                    if(Serial2.available()){
                        key = Serial2.read();
                        if (key == '4'){
                            break;
                        }
                    }
                }

            case 4: //採取フェーズ

            case 5: //発射フェーズ
                if(Serial2.available()){            //無線データに受信があるか
                        char key = Serial2.read();      //受信データの1文字を読み込む
                        if(key == 'l'){

                            Serial2.write("WARNING: The firing code has been entered.\n");
                            Serial2.write("WARNING: Are you sure you want to fire it?\n");
                            Serial2.write("WARNING: Press the y key to allow firing.\n");

                            while(true){

                                if(Serial2.available()){
                                    char key = Serial2.read();
                                    
                                    if(key == 'y'){
                                        Serial2.write("COUNTDOWN: 3\n");
                                        delay(1000);
                                        Serial2.write("COUNTDOWN: 2\n");
                                        delay(1000);
                                        Serial2.write("COUNTDOWN: 1\n");
                                        delay(1000);

                                        Serial2.write("LAUCHING: 9V voltage is output.\n");
                                        digitalWrite(lauchc, HIGH); //オン

                                        delay(outputsecond * 1000);

                                        Serial2.write("LAUCHING: 9V voltage is stop.\n");
                                        digitalWrite(lauchc, LOW); //オフ
                                        break;
                                    }else if (key == 'n'){
                                        Serial2.write("****** Ignition operation canceled ******\n");
                                        break;
                                    }
                                }
                            }
                        }else if (key == 'e') //緊急停止用
                        {
                            digitalWrite(lauchc, LOW);
                            Serial2.write("WARNING: The EMERGENCY code has been entered\n");
                        }
                    }

}


void readcommand(){
    // 受信データがあった時だけ、処理を行う
    if (Serial2.available()) {       // 受信データがあるか？
        key = Serial2.read();            // 1文字だけ読み込む
}

void caculator(){
    
}