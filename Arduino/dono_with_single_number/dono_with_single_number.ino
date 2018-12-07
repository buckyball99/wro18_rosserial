
#include <pinDefsAutoNew.h>
#include <sra128.h>
//#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <ArduinoHardware.h>
#include <StandardCplusplus.h>
#include <Wire.h>
#include <std_msgs/Float32.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/MultiArrayLayout.h>
#include<std_msgs/MultiArrayDimension.h>
#include<VL53L0X.h>
#include "mcp.h"
#include <sra128.h>
#include <stdio.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <iostream>

#define frontSensorCS PF0
#define backSensorCS PF1


ros::NodeHandle nh;
VL53L0X sensor;
float tof_reading;
float sensorarray[3][8];
float sensorsubscribe[24];

std_msgs::Float32MultiArray sensormultiarray;






int frontSensorReading[8];
int backSensorReading[8];

long sensor_time;

///std_msgs::Int32MultiArray::ConstPtr& array

void sensorcallback(const std_msgs::Float32MultiArray::constPtr &sensormultiarray){
    int k=0;
    for(std::vector<float>::const_iterator it=sensormultiarray->data.begin(); it!= sensormultiarray.end();++it ) 
    {
      sensorsubscribe[k]= *it;
        k++;
      
     }

  }

void readFrontSensor()
{
  for(int i =0;i<8;i++)
  {
    frontSensorReading[i] = getSensorReading(frontSensorCS,i);
  }
}

void readBackSensor()
{
  for(int i =0;i<8;i++) 
  {
    backSensorReading[i] = getSensorReading(backSensorCS,i);
  }
}
ros::Publisher pub_tof("Atmegasensors", &sensormultiarray);
ros::Subscriber<std_msgs::Float32MultiArray> sub_tof ("Atmegasensors", &sensorcallback);

int main() {
  // put your setup code here, to run once:
 Serial.begin(57600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  spiMasterInit();
   nh.initNode();
  //nh.subscribe(sub);
  sensormultiarray.layout.dim = (std_msgs::MultiArrayDimension *)malloc (sizeof(std_msgs::MultiArrayDimension) *2);
  sensormultiarray.layout.dim[0].label = "Height";
  sensormultiarray.layout .dim[1].label = "Width";
  sensormultiarray.layout.dim[0].size = 3;
  sensormultiarray.layout.dim[1].size = 8;
  sensormultiarray.layout.dim[0].stride = 3*8;
  sensormultiarray.layout.dim[1].stride = 8;
  sensormultiarray.layout.data_offset = 0;
  sensormultiarray.data= (float *)malloc(sizeof(float)*24);
  sensormultiarray.data_length= 24;
  nh.advertise(pub_tof);

   sensor.startContinuous();

 


while(1) {

  readFrontSensor();
  readBackSensor();
  
  // put your main code here, to run repeatedly:
       for(int i=0;i < 1; i++){
        for(int j=0; j<8;j++){
          sensorarray[i][j]= sensor.readRangeContinuousMillimeters();
          }

        }

       for(int i=1;i<2; i++){
            for(int j=0;j<8;j++){
             sensorarray[i][j]= frontSensorReading[j];    
              
          }
       }

       for(int i=2;i<3;i++){
            for(int j=0;j<8;j++){
             sensorarray[i][j]= backSensorReading[j]; 
          }
        
        }

      for(int i =0;i<3;i++){
           for( int j=0;j<8;j++){
              sensormultiarray.data[j] = sensorarray[i][j];
            }
          
        }
  
       
        pub_tof.publish(&sensormultiarray);
        nh.spinOnce();
}



long sensor_time;

}



