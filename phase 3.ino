 if(!phase_state == 3){
  //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
  Serial2.Write("Phase3: transition completed\n");
  Serial2.Write("");
  phase_state = 3;
  int type = 1;
  int type_state = 1;
  }

  swich(type)

  case 1:

  if(!type_state == 1){
    //電流フェーズに入ったとき１回だけ実行したいプログラムを書く
    Serial2.Write("type1: transition completed\n");
    Serial2.Write("");
    phase_state = 2;
    St_Time = 3time + outputcutsecond * 1000;        //基準時間
    Serial2.write("WARNING: The cut-para code has been entered.\n");
    digitalWrite(cutparac, HIGH); //オン
    Serial2.write("WARNING: 9v voltage is output.\n");
  }

  3_1time = currentMillis;
  if(3_1time > StTime){
  digitalWrite(cutparac, LOW); //オフ
  Serial2.write("WARNING: 9v voltage is stop.\n");
  phase = 4;
  }


  case 2:

  if(!type_state == 2){
    //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
    Serial2.Write("type2: transition completed\n");
    Serial2.Write("");
    type_state = 3;
    i = 0;
    j = 0;
    PreAc = 0;         //1秒前の加速度を記憶
    differ = 0.1;      //移動平均の差
  }

  accel = accelsqrt;
  Acsum = 0;         //加速度5個の合計値
  Acave = 0;         //加速度5個の平均値
  Realdif = 0;       //1秒前との差を記憶する

  if (i < 5){          
    Accel[i] = accel;
    i = i + 1;
  }
  else( i >= 5 ){          //データが五個集まったとき
    Accel[i] = accel;
    for(j=i-4 ; j==i ; j++){
      Acsum = Acsum + Accel[j];
      i = i + 1;
    }
    Acave = Acsum / 5;
    Realdif = Preac - Acave;
    if( Realif < differ ){          //移動平均が基準以内の変化量だったた時
      phase = 4;
    }
  }
