#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;
//HardwareSerial Serial2(1);
int i=0;
int n=0;
int j=0;
float gpslat[10];
float sum_lat;
float gpslng[10];
float sum_lng;  
float GOAL_lat = 35.862857820;
float GOAL_lng = 139.607681275;
float v_initial= 38.0;  //[m/s]
float g        = 9.80665;  //[m/s/s]
float delta_lng,GPS_lat,GPS_lng,distance,angle_radian,angle_degree;
void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial2.begin(9600);
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
     if(n==0){//10個のデータがたまるまで
      gpslat[i] = gps.location.lat();
      gpslng[i] = gps.location.lng();
      i++;
      if(i==10){
        for(j=0;j<10;j++){
         sum_lat = sum_lat + gpslat[j];
         sum_lng = sum_lng + gpslng[j];
        }
        GPS_lat=sum_lat/10;
        GPS_lng=sum_lng/10;
        delta_lng=GOAL_lng-GPS_lng;
        distance = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
        angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
        angle_degree = angle_radian*360.0/(2.0*M_PI);
        Serial.print("LAT:  "); Serial.println(GPS_lat, 9);
        Serial.print("LONG: "); Serial.println(GPS_lng, 9);
        Serial.print("DISTANCE[m]: "); Serial.println(distance,9);
        Serial.print("ANGLE[°]: "); Serial.println(angle_degree,9);
        n++;
        i=0;
        sum_lat=0;
        GPS_lat=0;
        sum_lng=0;
        GPS_lng=0;
        
      }
     }
     else{//10個のデータがたまった後
      for(j=0;j<9;j++){
       gpslat[j]=gpslat[j+1]; 
       gpslng[j]=gpslng[j+1];
      }
      gpslat[9]=gps.location.lat();
      gpslng[9]=gps.location.lng();
      for(j=0;j<10;j++){
       sum_lat = sum_lat + gpslat[j];
       sum_lng = sum_lng + gpslng[j];
      }
      GPS_lat=sum_lat/10;
      GPS_lng=sum_lng/10;
      delta_lng=GOAL_lng-GPS_lng;
      distance = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
      angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
      angle_degree = angle_radian*360.0/(2.0*M_PI);
      Serial.print("LAT:  "); Serial.println(GPS_lat, 9);
      Serial.print("LONG: "); Serial.println(GPS_lng, 9);
      Serial.print("DISTANCE[m]: "); Serial.println(distance,9);
      Serial.print("ANGLE[°]: "); Serial.println(angle_degree,9);
      sum_lat=0;
      GPS_lat=0;
      sum_lng=0;
      GPS_lng=0;
     }
    }
  }
}
