void setup() {
  Serial.begin(9600);
}

void loop() {
  for(int n=1 ; n>=0 ; n++){
    if(n%5==0){
      Serial.println("ばかやろう");
      delay(2000);
    }
    else
    {
      Serial.println("なんでやねん");
      delay(2000);
    }
  }
}
