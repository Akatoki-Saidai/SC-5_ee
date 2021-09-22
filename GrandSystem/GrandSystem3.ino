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
  Serial.println("Initializing SD card...");

  if(!SD.begin(4)){
    Serial.println("Initialization failed.");
    while(1);
  }else{
    Serial.println("Wirinig is corrct and card is present.");
  }

 file = SD.open("test.bin",FILE_WRITE);
 if(file){
    Serial.println("done");
  }else{
    Serial.println("error...");
  }
}  

void loop() {
  if(Serial2.available()){
   　for(int xx=0; xx>=0; xx++){
      for (int x=0; x <= 2; x++){
        Serial2.listen();
        byte a = Serial2.read();
        byte b = a<<8;
        byte c = Serial2.read();
        int16_t d = b | c;

        Serial.write(int(d));
        
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
    }
    
   label:
    for(int yy=0; yy>=0; yy++)
     for (int y=0; y < 14; y++){
       byte e = Serial2.read();
       byte f = e<<8;
       byte g = Serial2.read();
       byte h = f|g<<8;
       byte i = Serial2.read();
       byte j = h|i<<8;
       byte k = Serial2.read();
       byte l = j|k<<8;
       byte m = Serial2.read();
       byte n = l|m<<8;
       byte o = Serial2.read();
       byte p = n|o<<8;
       byte q = Serial2.read();
       byte r = p|q<<8;
       byte s = Serial2.read();
       int64_t t = r|s<<8;

      if(file){
          file.write(t);
        }
       
       unsigned long curr = millis();
       if((curr - prev) >= interval){
         Serial.write((int)t);
         prev = curr;
       }
       

      if(y > 0){
        continue;
          }
       int u = (t,DEC);
       
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
