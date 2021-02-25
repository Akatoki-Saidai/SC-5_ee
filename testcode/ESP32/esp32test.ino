
void setup() {
  Serial.begin(115200);
  pinMode( 2, OUTPUT );        // sets the digital pin 2 as output
  }

void loop() {
  
    digitalWrite(2, LOW); 
    Serial.print("LOW");
    delay(500);
    
    digitalWrite(2, HIGH );   // sets the LED off
    Serial.println("HIGH");
    delay(500);
}
