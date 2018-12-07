
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
  int  botforward;
  int  botright;
  int  botbackward;
  int  botleft;
  
  void printData() { 
  }


  void assignDirections(){
       botforward = directionF/1000;
       botright = (directionF/100)%10;
       botbackward = (directionF/10)%10;
       botleft = (directionF%10);
    }
  void assignPwm(){
     
      PWMF = pwmF;
      PWMR = pwmR;
      PWMB = pwmB;
      PWML = pwmL;
    }

  void BOT_STOP()
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 1;
  //
  MOTORBF = 1;// BACK
  MOTORBB = 1;
  //
  MOTORLF = 1; // LEFT
  MOTORLB = 1;
  //
  MOTORFF = 1; // FRONTT
  MOTORFB = 1;

}
void BOT_FORWARD()
{
  MOTORRF = 0;// RIGHT
  MOTORRB = 1;
  //
  //MOTORBF = 0;// BACK
  //MOTORBB =1;
  //
  MOTORLF = 0; // LEFT
  MOTORLB = 1;
  //
  //  MOTORFF = 0; // FRONTT
  // MOTORFB = 1;

}

void BOT_BACKWARD()
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;
  //
  //MOTORBF = 0;// BACK
  //MOTORBB =1;
  //
  MOTORLF = 1; // LEFT
  MOTORLB = 0;
  //
  //  MOTORFF = 0; // FRONTT
  // MOTORFB = 1;

}

void SPOT_LEFT()              // chNGE LATER
{
  MOTORRF = 0;// RIGHT
  MOTORRB = 1;

  MOTORBF = 0;// BACK     m
  MOTORBB = 1;

  MOTORLF = 1; // LEFTb b
  MOTORLB = 0;

  MOTORFF = 1; // FRONTT
  MOTORFB = 0;
}
void SPOT_RIGHT()               // change later
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;

  MOTORBF = 1;// BACK
  MOTORBB = 0;

  MOTORLF = 1; // LEFT
  MOTORLB = 0;

  MOTORFF = 0; // FRONTT
  MOTORFB = 1;

}
  
  
};
 motorData md;



#define _1_MESSAGE 'D'
#define _2_MESSAGE 'P'
#define _3_MESSAGE 'a'
#define _4_MESSAGE 'b'
#define _5_MESSAGE 'c'
#define _6_MESSAGE_end'd'
std_msgs::String str_msg;

void decrypt_message(String message)
{ String str_obj;
  

  int itr=0;
  int index = message.indexOf(_1_MESSAGE)+1;
  int length = message.indexOf(_2_MESSAGE);
 
  str_obj= message.substring(index,length);
  md.directionF =str_obj.toInt();

    itr=0;
   index = message.indexOf(_2_MESSAGE)+1;
   length = message.indexOf(_3_MESSAGE); 
   str_obj= message.substring(index,length);
   md.pwmF = str_obj.toInt();
  
    itr=0;
   index = message.indexOf(_3_MESSAGE)+1;
   length = message.indexOf(_4_MESSAGE);
     str_obj= message.substring(index,length);
  md.pwmR = str_obj.toInt();

    itr=0;
    index = message.indexOf(_4_MESSAGE)+1;
     length = message.indexOf(_5_MESSAGE);
       str_obj= message.substring(index,length);
    md.pwmB = str_obj.toInt();

       itr=0;
    index = message.indexOf(_5_MESSAGE)+1;
     length = message.indexOf(_6_MESSAGE_end);
       str_obj= message.substring(index,length);
    md.pwmL = str_obj.toInt();

  
  
 // md.printData();
  
  
  }
  void dataCallback(const std_msgs::String& msg)
{
  String m;
  m=msg.data;
  decrypt_message(m);
  md.assignPwm();
  md.assignDirections();
  if(md.pwmF==0 && md.pwmR==0 && md.pwmB==0 && md.pwmL==0){
      md.BOT_STOP();
    }

  if(md.botforward==1){
    MOTORLF=0;
    MOTORLB=1;
    }
  else
    {MOTORLF=1;
    MOTORLB=0;
    }
  if(md.botright==1){
    MOTORFF=1;
    MOTORFF=0;
     }
  else
    {MOTORFF=0;
    MOTORFF=1;
    }
  if(md.botbackward==1){
    MOTORRF=0;
    MOTORRF=1;
  }
  else{
    MOTORRF=1;
    MOTORRF=0;
    }
  if(md.botleft==1)
  {
    MOTORBF=0;
    MOTORBF=1;
  }
  else{
    MOTORBF=1;
    MOTORBF=0;
    }

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

