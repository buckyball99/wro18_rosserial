
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
double weightedSumBack, weightedSumFront, sumback, sumfront;
int previouserrorfront = 0, previouserrorback = 0;


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
///std_msgs::Int32MultiArray::ConstPtr& array

string int_array_to_string(int int_array[], int size_of_array) {
  std::ostringstream os("");
  for (int temp = 0; temp < size_of_array; temp++)
    os << int_array[temp];
  return os.str();
}

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

    frontsensorVal[j] = constrain(map(frontSensorReading[j], 100, 3000, 0, 10000), 0, 14000); // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");
    if (frontsensorVal[j] > 5000 )
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

    backsensorVal[j] = constrain(map(backSensorReading[j], 100, 3000, 0, 10000), 0, 10000); // 20  150 1000 0

    if (backsensorVal[j] > 7000)
    {
      backdigital[j] = 1;
    }

    else
      backdigital[j] = 0;


  }

}

void calculate_pid()
{

 

  //  Serial.print(frontpwm);
  //    Serial.print(\t);
  //    Serial.print(backpwm);
  //    Serial.println();

  for (int j = 0; j < 8; j++)
  {
    if (frontSensorReading[j] > 0)
    {
      weightedSumFront += (double)(frontsensorVal[j]) * ((j));
      sumfront += frontsensorVal[j];
    }
  }

  // Serial.print(weightedSumFront);
  // Serial.print("\t");
  // Serial.println(sumfront);

  if (sumfront != 0)
  {

    posfront = (double)weightedSumFront / sumfront * 10;
  }
  //  Serial.print(posfront);
  for (int j = 0; j < 8; j++)
  {

    if (backSensorReading[j] > 0)
    {
      weightedSumBack += (long)(backsensorVal[j]) * ((j) * 10);
      sumback += backsensorVal[j];
    }
  }

  if (sumback != 0)
  {
    posback = weightedSumBack / sumback;
  }
  //    Serial.print("\t");
  //    Serial.println(posback);
  fronterror = posfront - accuratefront;
  //  backerror = map( (posback - accurateback),-7,4,-10,10);
  backerror = accurateback - posback;
  /*  Serial.print(fronterror); */
  //    Serial.print("\t");
  //    Serial.println(backerror); // -7 se 4 symetric not



  frontpwmold = fronterror;
  backpwmold = 0 - backerror;

  //  Serial.print(frontpwmold);
  //  Serial.print("\t");
  //  Serial.println(backpwmold);

  // Serial.print(posfront);
  // Serial.print("\t");
  // Serial.println(posback);
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


  if (allWhiteFlagfront == 1)

  {
    if (previousfrontdigital[0] == 1)
    { frontpwmold = -22;
      Serial.println("a");
    }// changed it posi and nega
    else if (previousfrontdigital[7] == 1)
    { frontpwmold = 22;
      Serial.println("b");
    }
  }
  //  fronterror = posfront - accuratefront;
  //  allWhiteFlagfront = 0;

  if (backdigital[0] == 0 && backdigital[1] == 0 && backdigital[2] == 0 && backdigital[3] == 0 && backdigital[4] == 0 && backdigital[5] == 0 && backdigital[6] == 0 && backdigital[7] == 0)
  {
    allWhiteFlagback = 1;
  }
  else
  {
    for (int f = 0; f < 8; f++)
    {
      previousbackdigital[f] = backdigital[f];
    }
  }

  if (allWhiteFlagback == 1)

  {
    if (previousbackdigital[7] == 1 )
    { backpwmold = 26;
      Serial.println("d");
    }
    else if (previousbackdigital[0] == 1)
    { backpwmold = -26;
      Serial.println("c");
    }
  }

  //  fronterror = posfront - accuratefront;
  //  backerror = posback - accurateback;

  allWhiteFlagfront = 0;
  allWhiteFlagback = 0;

  if (frontpwmold < 0)
  {
    // Serial.println("front dor changeed");
    frontpwm = abs(frontpwmold);
    MOTORFF = 1; // FRONTT   CHANGE DOR OF MOTOR
    MOTORFB = 0;
  }

  else {
    frontpwm = abs(frontpwmold);
    MOTORFF = 0; // FRONTT   CHANGE DOR OF MOTOR
    MOTORFB = 1;
  }
  if (backpwmold < 0)
  {
    // Serial.println ("back  dor changeed");

    backpwm = abs(backpwmold);
    MOTORBF = 1;   //BACK   CHANGE DOR OF MOTOR
    MOTORBB = 0;
  }
  else {
    backpwm = abs(backpwmold);
    MOTORBF = 0; // back    CHANGE DOR OF MOTOR
    MOTORBB = 1;
  }
 

  frontpwmd = map(frontpwm, 0, 22, 80, 105);
  backpwmd = map(backpwm, 0, 26, 80, 105);
  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;
  previouserrorfront = fronterror;
  previouserrorback = backerror;
  /*     Serial.print(frontpwmd);
        Serial.print("\t");
        Serial.print(backpwmd);
        Serial.println();*/







}










ros::Publisher pub("Atmega", &sensorstring);
//ros::Subscriber<std_msgs::Float32MultiArray> sub_tof ("Atmegasensors", &sensorcallback);

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
  readFrontSensor();
  readBackSensor();
  calculatefrontSensorValues();
  calculatebackSensorValues();
  readdigisensorleft();
  Tof_detection();
  
  
  
  // put your main code here, to run repeatedly:
//       for(int i=0;i < 1; i++){
//        for(int j=0; j<8;j++){
//          sensorarray[i][j]= sensor.readRangeContinuousMillimeters();
//          }
//
//        }
//
//       for(int i=1;i<2; i++){
//            for(int j=0;j<8;j++){
//             sensorarray[i][j]= frontSensorReading[j];    
//              
//          }
//       }
//
//       for(int i=2;i<3;i++){
//            for(int j=0;j<8;j++){
//             sensorarray[i][j]= backSensorReading[j]; 
//          }
//        
//        }
//
//      for(int i =0;i<3;i++){
//           for( int j=0;j<8;j++){
//              sensormultiarray.data[j] = sensorarray[i][j];
//            }
//          
//        }
//  
        object.append("f");
        for(int i=0;i<8;i++){
          f[i]=frontdigital[i];
  
          }

        object.append(int_array_to_string(f,8));
        object.append("b");       
        for(int i=0;i<8;i++){
          b[i]= backdigital[i];
          }
        object.append(int_array_to_string(b,8));

       
        object.append("w");

        w= fronterror+100;
        object.append(int_data_to_string(w));

        object.append("s");

        s=backerror+100;
        object.append(int_data_to_string(s));
        object.append("l");
         l=digisensorleft;
        object.append(int_data_to_string(l));
                
        object.append("r");
        r=0;
        object.append(int_data_to_string(r));
        object.append("t");
        if(reading_analog<8000)
          t= reading_analog;
        else
          t=0;
       object.append(int_data_to_string(t));

        object.append("y");
        y=45;
        object.append(int_data_to_string(y));
       object.append("e");

     //   object = "f215151fsfs5f1515fsfsff";
       sensorstring.data=object.c_str();
        pub.publish(&sensorstring);
        nh.spinOnce();



}






