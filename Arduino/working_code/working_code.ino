
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
#include <std_msgs/String.h>
#include<VL53L0X.h>
#include "mcp.h"
#include <sra128.h>
#include <stdio.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <iostream>
#include<sstream>
using namespace std;
#define frontSensorCS PF0
#define backSensorCS PF1


ros::NodeHandle nh;
VL53L0X sensor;
float tof_reading;
float sensorarray[3][8];
float sensorsubscribe[24];
int f[8],b[8],w,s,l,r,t,y,e;
std_msgs::String sensorstring;
int frontSensorReading[8], backdigital[8],frontsensorVal[8];
int backSensorReading[8],frontdigital[8],backsensorVal[8];
string object;
int digisensorleft,reading_analog;


string int_data_to_string(int data){
  std::ostringstream oss("");
  oss<< data;
  return oss.str();
}

void Tof_detection(){
  
  reading_analog= sensor.readRangeContinuousMillimeters(); 
 //Serial.print("Reading: \t");
 // Serial.println(reading_analog);
  }

  ros::Publisher pub("Atmega", &sensorstring);


  void setup() {
  // put your setup code here, to run once:
 Serial.begin(57600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  spiMasterInit();
   nh.initNode();
  //nh.subscribe(sub);
 
  nh.advertise(pub);

   sensor.startContinuous();
}

void loop(){
 object = "";
 Tof_detection();
  
object.append("t");
        t= reading_analog;
       object.append(int_data_to_string(t));


        sensorstring.data=object.c_str();
        pub.publish(&sensorstring);
        nh.spinOnce();



}

