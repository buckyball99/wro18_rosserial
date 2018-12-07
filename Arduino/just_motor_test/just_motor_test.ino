
#include <stdio.h>
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
//#include <sra128.h>
#include <stdio.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <iostream>
#include<sstream>
#include <stdlib.h>
using namespace std;

#define frontSensorCS PF0
#define backSensorCS PF1

ros::NodeHandle nh;

void setdirFront(){
  MOTORFF= 1;
  MOTORFB =0;
 }

void setdirRight(){
  MOTORRF= 0;
  MOTORRB =1;
 }
void setdirBack(){
  MOTORBF= 0;
  MOTORBB =1;
 }

void setdirLeft(){
  MOTORLF= 0;
  MOTORLB =1;
 }

 #define _1_MESSAGE 'P'
 #define _2_MESSAGE 'E'

void decrypt(String message){
   String str_obj;
  int index =  message.indexOf(_1_MESSAGE) +1 ;
  int length =  message.indexOf(_2_MESSAGE);
   str_obj = message.substring(index,length);
   PWMF = str_obj.toInt();
  }

void dataCallback(const std_msgs::String& msg)
{
  String m;
  m = msg.data;
  decrypt(m);
  setdirFront();

}

ros::Subscriber<std_msgs::String>sub("AtmegaIn", dataCallback);


void setup() {
  Serial.begin(57600);

  pwm1_init();
  pwm2_init();
  pwm3_init();
  DDRC = 0xFF;
  spiMasterInit();
  nh.initNode();
  nh.subscribe(sub);



}

void loop(){
  nh.spinOnce();
  
  }

 

