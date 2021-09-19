#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>

File file;
SoftwareSerial Serial2(2,3);
unsigned long prev, next, interval;

void setup() {
  
  prev = 0;
  interval = 2000;
  
  Serial.begin(115200);
  Serial2.begin(115200);
  while(!Serial);
    Serial.println("Cannot mobilize this system.")
  }
  //SD test
  Serial.println("Initializing SD card...");

  if(!SD.begin(4)){
    Serial.println("Initialization failed.");
    while(1);
  }else{
    Serial.println("Wirinig is corrct and card is present.");
  }

  file = SD.open("test.bin",FILE_WRITE);

  if(file){
    Serial.println("Writing to test.bin...");
    file.write("testing 1,2,3.\n");
    file.close();
    Serial.println("done");
  }else{
    Serial.println("error...");
  }
  file = SD.open("test.bin");
  if(file){
    Serial.println("test.bin:");
    Serial.println(file.read());
    file.close();
  }else{
    Serial.println("error opening test.bin");
  }
  
 Serial.println("SD test ended.");
 //SD test ended.

 file = SD.open("test.bin",FILE_WRITE);
 if(file){
    Serial.println("done");
  }else{
    Serial.println("error...");
  }
}  

void loop() {
  if(Serial2.available()){
      for (int x=0; x <= 2; x++){
        Serial2.listen();
        byte a = Serial2.read();
        byte b = a<<8;
        byte c = Serial2.read();
        int16_t d = b | c;

        Serial.write(d);
        
        if(file){
          file.write(d);
        }

        if(x > 0){
          continue;
        }

        switch(d){
          case 1:
           Serial.println("Phase1:transition completed.");
           break;
          case 2:
           Serial.println("Phase2:transition completed.");
           break;
          case 3:
           Serial.println("Phase3:transition completed.");
           break;
          case 4:
           Serial.println("Phase4:transition completed.");
            
            goto label;
            
           break;
          case 5:
           Serial.println("Phase5:transition completed.");
           break;
          case 6:
           Serial.println("WARNING:The cut-para code has been entered.");
           break;
          case 7:
           Serial.println("WARNING:9v voltage is output."); 
           break;
          case 8:
           Serial.println("WARNING:9v voltage is stop.");
           break;
          case 9:
           Serial.println("Phase3: Process all completed. Enter '4 key.'");
           break; 
          default:;
        }
    }
   label:
    
     for (int y=0; y < 14; y++){
       byte e = Serial2.read();
       byte f = e<<56;
       byte g = Serial2.read();
       byte h = g<<48;
       byte i = Serial2.read();
       byte j = i<<40;
       byte k = Serial2.read();
       byte l = k<<32;
       byte m = Serial2.read();
       byte n = m<<24;
       byte o = Serial2.read();
       byte p = o<<16;
       byte q = Serial2.read();
       byte r = q<<8;
       byte s = Serial2.read();
       int64_t u = f | h | j | l | n | p | r;

      if(file){
          file.write((int)u);
        }
       
       unsigned long curr = millis();
       if((curr - prev) >= interval){
         Serial.println((int)u);
         prev = curr;
       }
       

      if(y > 0){
        continue;
          }
       
      switch(u){
        case 1:
         Serial.println("Phase1:transition completed.");
         break;
        case 2:
         Serial.println("Phase2:transition completed.");
         break;
        case 3:
         Serial.println("Phase3:transition completed.");
         break;
        case 4:
         Serial.println("Phase4:transition completed.");
         break;
        case 5:
         Serial.println("Phase5:transition completed.");
         break;
        case 6:
         Serial.println("WARNING:The cut-para code has been entered.");
         break;
        case 7:
         Serial.println("WARNING:9v voltage is output."); 
         break;
        case 8:
         Serial.println("WARNING:9v voltage is stop.");
         break;
        case 9:
         Serial.println("Phase3: Process all completed. Enter '4 key.'");
         break; 
         default:;
        }
     }
   }
}
