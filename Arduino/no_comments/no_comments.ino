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
VL53L0X sensor;
float tof_reading;
float sensorarray[3][8];
float sensorsubscribe[24];
int f[8], b[8], w, s, l, r, t, y, e;
std_msgs::String sensorstring;
int frontSensorReading[8], backdigital[8], frontsensorVal[8];
int backSensorReading[8], frontdigital[8], backsensorVal[8];
string object;
int digisensorleft, reading_analog;
double weightedSumBack, weightedSumFront, sumback, sumfront;
int previouserrorfront = 0, previouserrorback = 0;

bool shouldPublish = true;

int bot_said_left = 'y';
char condition = 'a';
int  previousfrontdigital[8], previousbackdigital[8];
int  allWhiteFlagback, accurateback = 35, accuratefront = 32 , backerror, posback, posfront, fronterror;
int frontjunction = 0, count;
int  frontpwmold, backpwmold;
int digisensor, previousdigisensor, previousdigisensorleft, previousfrontsensorVal[8];
int frontpwmd, backpwmd, leftpwm = 200, rightpwm = 200, allWhiteFlagfront;
int readVal, positionVal, accurateval = 29, accuratevalback = 25, previouserror1 , junctionno , yawerror1, yawerror2 = 0, lateralerror1 = 0, lateralerror2 = 0, totalerror1 = 0, totalerror2 = 0;
//int thresh = 50;


int previouserror, frontpwm, backpwm;
long sensor_time;

string int_array_to_string(int int_array[], int size_of_array) {
  std::ostringstream os("");
  for (int temp = 0; temp < size_of_array; temp++)
    os << int_array[temp];
  return os.str();
}

string int_data_to_string(int data) {
  std::ostringstream oss("");
  oss << data;
  return oss.str();
}




void Tof_detection() {

  reading_analog = sensor.readRangeContinuousMillimeters();
  //Serial.print("Reading: \t");
  // Serial.println(reading_analog);
}


void readdigisensorleft()
{
  if (bit_is_set(PIND, 2))
  {
    digisensorleft = 1;

  }
  else
  {
    digisensorleft = 0;
  }
}


void readFrontSensor()
{
  for (int i = 0; i < 8; i++)
  {
    frontSensorReading[i] = getSensorReading(frontSensorCS, i);

  }
}

void readBackSensor()
{
  for (int j = 7; j >= 0; j--)
  {
    backSensorReading[j] = getSensorReading(backSensorCS, (7 - j));

  }
}

void calculatefrontSensorValues()
{
  for (int j = 0; j < 8; j++)
  {

    frontsensorVal[j] = constrain(map(frontSensorReading[j], 100, 3000, 0, 1000), 0, 1000); // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");
    if (frontSensorReading[j] > 2200 )
    {
      frontdigital[j] = 1;
    }

    else
      frontdigital[j] = 0;


  }
}
void calculatebackSensorValues()
{
  for (int j = 0; j < 8; j++)
  {
    //    sensorVal[j] = map(sensorRaw[j], 0, 700, 0, 2000);    // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");

    backsensorVal[j] = constrain(map(backSensorReading[j], 100, 3000, 0, 1000), 0, 1000); // 20  150 1000 0

    if (backSensorReading[j] > 2200)
    {
      backdigital[j] = 1;
    }

    else
      backdigital[j] = 0;


  }

}

void calculate_pid()

{

  for (int j = 0; j < 8; j++)
  {
        weightedSumFront += (double)(frontsensorVal[j]) * ((j)*10);     
     if( frontdigital[j] == 1)
     {
      sumfront += frontsensorVal[j];
     }
           
  }


  if (sumfront != 0)
  {

    posfront = (double)weightedSumFront / sumfront;
  }
  //  Serial.print(posfront);
  for (int j = 0; j < 8; j++)
  {

   
      weightedSumBack += (long)(backsensorVal[j]) * ((j)*10 );
      if( backdigital[j] ==1)
      {
      sumback += backsensorVal[j];
      }
    
  }

  if (sumback != 0)
  {
    posback = weightedSumBack / sumback;
  }
  

  
  fronterror = posfront - accuratefront;

  backerror = accurateback - posback;
 



  frontpwmold = fronterror;
  backpwmold = 0 - backerror;
  
  if (frontdigital[0] == 0 && frontdigital[1] == 0 && frontdigital[2] == 0 && frontdigital[3] == 0 && frontdigital[4] == 0 && frontdigital[5] == 0 && frontdigital[6] == 0 && frontdigital[7] == 0)
  {
    allWhiteFlagfront = 1;

  }
  else
  {
    for (int g = 0; g < 8; g++)
    {
      previousfrontdigital[g] = frontdigital[g];
    }
  }
  allWhiteFlagfront = 0;
  allWhiteFlagback = 0;

  frontpwmd = map(frontpwm, 0, 22, 80, 105);
  backpwmd = map(backpwm, 0, 26, 80, 105);
  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;
  previouserrorfront = fronterror;
  previouserrorback = backerror;
}











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
  int Delay;

  void printData() {
  }


  void assignDirections() {
    botforward = (directionF)/1000;
    botright = ((directionF)/100) % 10;
    botbackward = ((directionF)/10) % 10;
    botleft = ((directionF) % 10);
  }
  void assignPwm() {

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
//  void BOT_FORWARD()
//  {
//    MOTORRF = 0;// RIGHT
//    MOTORRB = 1;
//    //
//    //MOTORBF = 0;// BACK
//    //MOTORBB =1;
//    //
//    MOTORLF = 0; // LEFT
//    MOTORLB = 1;
//    //
//    //  MOTORFF = 0; // FRONTT
//    // MOTORFB = 1;
//
//  }
//
//  void BOT_BACKWARD()
//  {
//    MOTORRF = 1;// RIGHT
//    MOTORRB = 0;
//    //
//    //MOTORBF = 0;// BACK
//    //MOTORBB =1;
//    //
//    MOTORLF = 1; // LEFT
//    MOTORLB = 0;
//    //
//    //  MOTORFF = 0; // FRONTT
//    // MOTORFB = 1;
//
//  }
//
//  void SPOT_LEFT()              // chNGE LATER
//  {
//    MOTORRF = 0;// RIGHT
//    MOTORRB = 1;
//
//    MOTORBF = 0;// BACK     m
//    MOTORBB = 1;
//
//    MOTORLF = 1; // LEFTb b
//    MOTORLB = 0;
//
//    MOTORFF = 1; // FRONTT
//    MOTORFB = 0;
//  }
//  void SPOT_RIGHT()               // change later
//  {
//    MOTORRF = 1;// RIGHT
//    MOTORRB = 0;
//
//    MOTORBF = 1;// BACK
//    MOTORBB = 0;
//
//    MOTORLF = 1; // LEFT
//    MOTORLB = 0;
//
//    MOTORFF = 0; // FRONTT
//    MOTORFB = 1;
//
//  }


};

motorData md;



#define _1_MESSAGE 'D'
#define _2_MESSAGE 'P'
#define _3_MESSAGE 'a'
#define _4_MESSAGE 'b'
#define _5_MESSAGE 'c'
#define _6_MESSAGE'd'
#define _7_MESSAGE_end 'm'
std_msgs::String str_msg;

void decrypt_message(String message)
{ String str_obj;


  int itr = 0;
  int index = message.indexOf(_1_MESSAGE) + 1;
  int length = message.indexOf(_2_MESSAGE);

  str_obj = message.substring(index, length);
  md.directionF = str_obj.toInt();

  itr = 0;
  index = message.indexOf(_2_MESSAGE) + 1;
  length = message.indexOf(_3_MESSAGE);
  str_obj = message.substring(index, length);
  md.pwmF = str_obj.toInt();

  itr = 0;
  index = message.indexOf(_3_MESSAGE) + 1;
  length = message.indexOf(_4_MESSAGE);
  str_obj = message.substring(index, length);
  md.pwmR = str_obj.toInt();

  itr = 0;
  index = message.indexOf(_4_MESSAGE) + 1;
  length = message.indexOf(_5_MESSAGE);
  str_obj = message.substring(index, length);
  md.pwmB = str_obj.toInt();

  itr = 0;
  index = message.indexOf(_5_MESSAGE) + 1;
  length = message.indexOf(_6_MESSAGE);
  str_obj = message.substring(index, length);
  md.pwmL = str_obj.toInt();
  index = message.indexOf(_6_MESSAGE)+1;
  length =message.indexOf(_7_MESSAGE_end);
  str_obj= message.substring(index,length);
  md.Delay = str_obj.toInt();
}
void dataCallback(const std_msgs::String& msg)
{
  String m;
  m = msg.data;
  decrypt_message(m);
  md.assignPwm();
  md.assignDirections();

  if (md.botforward == 1) {
    MOTORFF = 0;
    MOTORFB = 1;
  }
  else
  { MOTORFF = 1;
    MOTORFB = 0;
  }



  if (md.botright == 1) {
    MOTORRF = 1;
    MOTORRB = 0;
  }
  else {
    MOTORRF = 0;
    MOTORRB = 1;
  }


  if (md.botbackward ==1)
  {
    MOTORBF = 1;
    MOTORBB = 0; }
  if (md.botleft == 1) {
   
    MOTORLF = 0;
    MOTORLB = 1;
  }
  else
  { MOTORLF = 1;
    MOTORLB = 0;
  }
  if (md.pwmF == 0 && md.pwmR == 0 && md.pwmB == 0 && md.pwmL == 0) {
    md.BOT_STOP();
  }
}
ros::Subscriber<std_msgs::String>sub("AtmegaIn",dataCallback);
ros::Publisher pub("AtmegaOut", &sensorstring);

void setup() 
{
  Serial.begin(57600);

  Wire.begin();
  DDRA = 0X00;
  DDRB=0xFF;
  PORTB=0xFF;
  PORTA |= (1<<PA5);                       
  DDRC = 0xFF;
  DDRD |= (1<<PD6)|(1<<PD7);
  PORTD|=(1<<PD6)|(1<<PD7);

  sensor.init();
  sensor.setTimeout(500);
  spiMasterInit();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
 // nh.getHardware()->setBaud(2000000);
 
  sensor.startContinuous();
  pwm1_init();
 
  pwm3_init();

}

void loop()
{ 
  object = "";
  readFrontSensor();
  readBackSensor();
  calculatefrontSensorValues();
  calculatebackSensorValues();
  readdigisensorleft();
  Tof_detection();
  calculate_pid();


  object.append("f");
  for (int i = 0; i < 8; i++) {
    f[i] = frontdigital[i];

  }

  object.append(int_array_to_string(f, 8));
  object.append("b");
  for (int i = 0; i < 8; i++) {
    b[i] = backdigital[i];
  }
  object.append(int_array_to_string(b, 8));


  object.append("w");

  w = fronterror + 100;
  object.append(int_data_to_string(w));

  object.append("s");

  s = backerror + 100;
  object.append(int_data_to_string(s));
  object.append("l");
  l = digisensorleft;
  object.append(int_data_to_string(l));

  object.append("r");
  r = 0;
  object.append(int_data_to_string(r));
  object.append("t");
  if(reading_analog<8000)
  t = reading_analog;
  else
  t=0;
  object.append(int_data_to_string(t));

  object.append("y");
  y = 45;
  object.append(int_data_to_string(y));
  object.append("e");

  //   object = "f215151fsfs5f1515fsfsff";
  sensorstring.data = object.c_str();


    
 
  pub.publish(&sensorstring);
  
  
  nh.spinOnce();



}

