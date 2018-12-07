
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
#include <sra128.h>
#include <stdio.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <iostream>
#include<sstream>
#include <stdlib.h>
using namespace std;
ros::NodeHandle nh;

struct motorData {


  int directionF;
  int directionR;
  int directionB;
  int directionL;
  int pwmF;
  int pwmR;
  int pwmB;
  int pwmL;

  void printData() {


    cout << "directionFront:" << directionF << endl;
    cout << "pwmFront:" << pwmF << endl;
    cout << "pwmRight:" << pwmR << endl;
    cout << "pwmBack:" << pwmB << endl;
    cout << "pwmLeft:" << pwmL << endl;





  }
};



#define _1_MESSAGE 'D'
#define _2_MESSAGE 'P'
#define _3_MESSAGE 'a'
#define _4_MESSAGE 'b'
#define _5_MESSAGE 'c'
#define _6_MESSAGE_end'd'
std_msgs::String str_msg;

void decrypt_message(string message)
{ String str_obj;
  motorData md;

  int itr=0;
  int index = message.find(_1_MESSAGE)+1;
  int length = message.find(_2_MESSAGE)-index;
 
  str_obj= message.substr(index,length);
  md.directionF =str_obj.toInt;

    itr=0;
   index = message.find(_2_MESSAGE)+1;
   length = message.find(_3_MESSAGE);
    md.pwmF = stoi(message.substr(index,length));
  
    itr=0;
   index = message.find(_3_MESSAGE)+1;
   length = message.find(_4_MESSAGE);
  md.pwmR = stoi(message.substr(index,length));

    itr=0;
    index = message.find(_4_MESSAGE)+1;
     length = message.find(_5_MESSAGE)- index;
    md.pwmB = stoi(message.substr(index,length));

       itr=0;
    index = message.find(_5_MESSAGE)+1;
     length = message.find(_6_MESSAGE_end)- index;
    md.pwmL = stoi(message.substr(index,length));

  
  
  md.printData();
  
  
  }
  void dataCallback(const std_msgs::String& msg)
{
  string m;
  m=msg.data;
  decrypt_message(m);
  cout<<"ooooooooooooooooooooooooooo"<<endl;

} 

ros::Subscriber<std_msgs::String>sub("Atmega",dataCallback);
ros::Publisher pub("Atmega",&str_msg);

void setup(){
  Serial.begin(57600);

 nh.initNode();
 nh.subscribe(sub);
  
  }

void loop(){
  nh.spinOnce();
  
  delay(1);
  
  }

