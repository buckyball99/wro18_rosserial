#include <Wire.h>
#include <VL53L0X.h>
#include <pinDefsAutoNew.h>
#include <sra128.h>
#include <io128.h>

float Tof_reading;
VL53L0X sensor;
void Tof_detection(){
  
  reading_analog= sensor.readRangeContinuousMillimeters(); 
  Serial,print("Reading: /t");
  Serial.println(reading_analog);
  }

 void Tof_convert(){
  if(reading_analog <= 50)
     reading_digital=1;

   else if (read_analog >= 80)
      reading_digital=0;
  
  }
int main(){
  // put your setup code here, to run once:
  sensor.startContinuous();
  Serial.begin(115200);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);

while(1){
  // put your main code here, to run repeatedly:
  Tof_detection();
  Tof_convert();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
 }

}
