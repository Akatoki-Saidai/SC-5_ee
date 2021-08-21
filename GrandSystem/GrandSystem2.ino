#include <SD.h>
#include <SPI.h>

File file;

char*filename="TEST.TXT";

void setup() {
  Serial.begin(115200);
  
    file = SD.open(filename, FILE_WRITE);
    if (file) {
      
      file.print(10,DEC);
      file.println(":test");
      file.close();
    }else{
       }
}

void loop() {
  if(Serial.available()){
    for (int z=0; z < 64; z++ /*64bit通信が始まるまで*/){
      for (int x=0; x <= 2; x++){
        byte a = Serial.read();
        byte b = a<<8;
        byte c = Serial.read();
        int16_t d = b | c;

        Serial.println(d);
        if(file){
          file.println("d");
          file.close();
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

     for (int y=0; y < 14; y++){
       byte e = Serial.read();
       byte f = e<<56;
       byte g = Serial.read();
       byte h = g<<48;
       byte i = Serial.read();
       byte j = i<<40;
       byte k = Serial.read();
       byte l = k<<32;
       byte m = Serial.read();
       byte n = m<<24;
       byte o = Serial.read();
       byte p = o<<16;
       byte q = Serial.read();
       byte r = q<<8;
       byte s = Serial.read();
       int64_t u = f | h | j | l | n | p | r;

      if(file){
          file.println("u");
          file.close();
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
