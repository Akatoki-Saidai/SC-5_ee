void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
}
int n=1;
void loop() {
  // put your main code here, to run repeatedly:
 {
    if (n%10==0){
      Serial.println("なんでやねん");
    }
    else {
      Serial.println("ばかやろう");
    }
    n=n+1;
    delay(2000);
    }
}
