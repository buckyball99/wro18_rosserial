#include <StandardCplusplus.h>
#include <utility.h>
#include <system_configuration.h>
#include <unwind-cxx.h>


#include <stdio.h>
#include <pinDefsAutoNew.h>
#include <sra128.h>
//#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <ArduinoHardware.h>
//#include <StandardCplus/plus.h>
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

#define frontSensorCS PF0
#define backSensorCS PF1


ros::NodeHandle nh;
VL53L0X sensor;
float tof_reading;
float sensorarray[3][8];
float sensorsubscribe[24];
int f[8], b[8], w, s, l, r, t, y, e;
std_msgs::String sensorstring;
string object;
const int pingPin = 7; //E7

long duration;
double cm;
int reading_analog;


bool shouldPublish = true;

//int  previousfrontdigital[8], previousbackdigital[8],frontSensorReading[8], backdigital[8], frontsensorVal[8];
//int  allWhiteFlagback, accurateback = 35, accuratefront = 32 , backerror, posback, posfront, fronterror;
//int frontjunction = 0, count;
//int  frontpwmold, backpwmold;
//int digisensor, previousdigisensor, previousdigisensorleft, previousfrontsensorVal[8];
//int frontpwmd, backpwmd, leftpwm = 200, rightpwm = 200, allWhiteFlagfront;
//int readVal, positionVal, accurateval = 29, accuratevalback = 25, previouserror1 , junctionno , yawerror1, yawerror2 = 0, lateralerror1 = 0, lateralerror2 = 0, totalerror1 = 0, totalerror2 = 0;
//int thresh = 50;


int frontSensorReading[8];
int previouserrorfront = 0, previouserrorback = 0;




int frontdigital[8], backdigital[8], previousfrontdigital[8], previousbackdigital[8];
int backSensorReading[8], allWhiteFlagback, accurateback = 3500, accuratefront = 3500 , backerror, posback, posfront, fronterror, frontsensorVal[8], backsensorVal[8];
double weightedSumBack, weightedSumFront, sumback, sumfront;

//long int averageFrontBlack[8] = {3018 , 3019 , 3017 , 2883 , 2927  ,2346,  2849,  2998  };
//long int averageFrontWhite[8] = {709 ,1139 ,628 ,391 ,230 ,193, 216 ,472};
//long int averageBackBlack[8] = {2109 , 2811  ,2966 , 2996,  2998 , 2587 , 2998 , 2999   };
//long int averageBackWhite[8] = {212 ,172, 220 ,221 ,241, 213, 637 ,1060};
//245 217 160 220 232 197 221 529 Back(Fixed):2840  3011  2347  ,3011 , 2959 , 2744 , 2958  ,2959  

//1088  , 1020  , 977  , 1151  , 952  , 1029  , 1189  , 1329  , Back(Fixed):1720 , 1769 , 1823 , 2669 , 1985 , 2156 , 2469 , 2767 , 
//Back Average:-- White(Fixed):431 , 599 , 653 , 218 , 525 , 181 , 1298 , 1440 , Back(Fixed):3069 , 3068 , 3068 , 3068 , 3063 , 2986 , 3063 , 3063 ,



long int averageFrontBlack[8] = {3058 , 2649 , 2759 , 2829 , 2005 , 2204 , 2561 , 3042  };    
long int averageFrontWhite[8] = {852  , 179  , 190  , 192  , 564  , 712  , 916  , 1116  };  
long int averageBackBlack[8] = {3018 , 3069 , 3037 , 3063 , 2939 , 3008 , 3062 , 3062 };
long int averageBackWhite[8] = {255 , 573 , 944 , 235 , 786 , 200 , 1456 , 1600 };      





int  frontpwmold, backpwmold;
int digisensorleft,digisensorright, previousdigisensorleft, previousfrontsensorVal[8];
int frontpwmd, backpwmd, leftpwm = 250, rightpwm = 250, allWhiteFlagfront;

unsigned long   weightedSum = 0, sum = 0;
int frontpwm, backpwm;


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
void readdigisensorright()
{
  if (bit_is_set(PIND, 3))
  {
    digisensorright = 1;

  }
  else
  {
    digisensorright = 0;
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

    frontsensorVal[j] = constrain(map(frontSensorReading[j],averageFrontWhite[j], averageFrontBlack[j],0,1000), 0, 1000); // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");
    if (frontsensorVal[j] > 500 )
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

    backsensorVal[j] = constrain(map(backSensorReading[j], averageBackWhite[j], averageBackBlack[j], 0, 1000), 0, 1000); // 20  150 1000 0

    if (backsensorVal[j] > 500)
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
    weightedSumFront += (double)(frontsensorVal[j]) * ((j) * 1000);
   // if ( frontdigital[j] == 1)
    //{
      sumfront += frontsensorVal[j];
    //}

  }

  // Serial.print(weightedSumFront);
  // Serial.print("\t");
  // Serial.println(sumfront);

  if (sumfront != 0)
  {

    posfront = (double)weightedSumFront / sumfront;
  }
  //  Serial.print(posfront);
  for (int j = 0; j < 8; j++)
  {


    weightedSumBack += (double)(backsensorVal[j]) * ((j) * 1000 );
   // if ( backdigital[j] == 1)
   // {
      sumback += backsensorVal[j];
//    }

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



  frontpwmold =0 -fronterror;
  backpwmold =  backerror;

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
  //                                                                  commmented bh manav
  //
  //  if (allWhiteFlagfront == 1)
  //
  //  {
  //    if (previousfrontdigital[0] == 1)
  //    { frontpwmold = -22;
  //      Serial.println("a");
  //    }// changed it posi and nega
  //    else if (previousfrontdigital[7] == 1)
  //    { frontpwmold = 22;
  //      Serial.println("b");
  //    }
  //  }
  //  //  fronterror = posfront - accuratefront;
  //  //  allWhiteFlagfront = 0;
  //
  //  if (backdigital[0] == 0 && backdigital[1] == 0 && backdigital[2] == 0 && backdigital[3] == 0 && backdigital[4] == 0 && backdigital[5] == 0 && backdigital[6] == 0 && backdigital[7] == 0)
  //  {
  //    allWhiteFlagback = 1;
  //  }
  //  else
  //  {
  //    for (int f = 0; f < 8; f++)
  //    {
  //      previousbackdigital[f] = backdigital[f];
  //    }
  //  }
  //
  //  if (allWhiteFlagback == 1)
  //
  //  {
  //    if (previousbackdigital[7] == 1 )
  //    { backpwmold = 26;
  //      Serial.println("d");
  //    }
  //    else if (previousbackdigital[0] == 1)
  //    { backpwmold = -26;
  //      Serial.println("c");
  //    }
  //  }

  //  fronterror = posfront - accuratefront;
  //  backerror = posback - accurateback;

  allWhiteFlagfront = 0;
  allWhiteFlagback = 0;

  //  if (frontpwmold < 0)
  //  {
  //    // Serial.println("front dor changeed");
  //    frontpwm = abs(frontpwmold);
  //    MOTORFF = 1; // FRONTT   CHANGE DOR OF MOTOR
  //    MOTORFB = 0;
  //  }
  //
  //  else {
  //    frontpwm = abs(frontpwmold);
  //    MOTORFF = 0; // FRONTT   CHANGE DOR OF MOTOR
  //    MOTORFB = 1;
  //  }
  //  if (backpwmold < 0)
  //  {
  //    // Serial.println ("back  dor changeed");
  //
  //    backpwm = abs(backpwmold);
  //    MOTORBF = 1;   //BACK   CHANGE DOR OF MOTOR
  //    MOTORBB = 0;
  //  }
  //  else {
  //    backpwm = abs(backpwmold);
  //    MOTORBF = 0; // back    CHANGE DOR OF MOTOR
  //    MOTORBB = 1;
  //  }


  frontpwmd = map(frontpwm, 0, 3500, 90, 120);
  backpwmd = map(backpwm, 0, 3500, 90, 120);
  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;
  previouserrorfront = fronterror;
  previouserrorback = backerror;
  /*     Serial.print(frontpwmd);
        Serial.print("\t");
        Serial.print(backpwmd);
        Serial.println();*/
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
    botforward = (directionF) / 1000;
    botright = ((directionF) / 100) % 10;
    botbackward = ((directionF) / 10) % 10;
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
  index = message.indexOf(_6_MESSAGE) + 1;
  length = message.indexOf(_7_MESSAGE_end);
  str_obj = message.substring(index, length);
  md.Delay = str_obj.toInt();



  // md.printData();


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


  if (md.botbackward == 1)
  {
    MOTORBF = 1;
    MOTORBB = 0;
  }
  else
  {

    MOTORBF = 0;
    MOTORBB = 1;


  }



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


  //  int Button4 = PORTA & (1<<PA5);
  //  int Button5= PORTA & (1<<PA6);
  // if(Button4 = 1){
  //    MOTORRF = 1;// RIGHT
  //    MOTORRB = 0;
  //
  //    MOTORLF = 1; // LEFT
  //    MOTORLB = 0;
  //
  //    _delay_ms(2);
  //
  //
  //  }
}

ros::Subscriber<std_msgs::String>sub("AtmegaIn", dataCallback);
ros::Publisher pub("AtmegaOut", &sensorstring);

void setup()
{
  Serial.begin(57600);

  Wire.begin();
  DDRA = 0X00;
  DDRB = 0xFF;
  DDRG = 0x00;
  PORTB = 0xFF;
  PORTA |= (1 << PA5);
  DDRC = 0xFF;
  DDRD |= (1 << PD6) | (1 << PD7);
  PORTD |= (1 << PD6) | (1 << PD7);

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
double microsecondsToCentimeters(double microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 28.90173 / 2;
}


void ultraPing()
{
  
  pinMode(pingPin, OUTPUT);
  
  
    digitalWrite(pingPin, LOW);
  _delay_us(2);
  digitalWrite(pingPin, HIGH);
  _delay_us(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  _delay_ms(5);
   
}

void loop()
{
  
  ultraPing();
  object = "";
  readFrontSensor();
  readBackSensor();
  calculatefrontSensorValues();
  calculatebackSensorValues();
  readdigisensorleft();
  readdigisensorright();
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

  w = fronterror;
  object.append(int_data_to_string(w));

  object.append("s");

  s = backerror;
  object.append(int_data_to_string(s));
  object.append("l");
  l = digisensorleft;
  object.append(int_data_to_string(l));

  object.append("r");
  r = digisensorright;
  object.append(int_data_to_string(r));
  object.append("t");
  if (reading_analog < 8000)
    t = reading_analog;
  else
    t = 0;
  object.append(int_data_to_string(t));

  object.append("y");
  y = floor(cm*10);
  object.append(int_data_to_string(y));
  object.append("e");
  if(PING>0)
  {
    object.append("1");
   }
   else
   {
    object.append("0");
    }
//    object.append("f");
//     if(PORTG |=(1<<PG1))
//  {
//    object.append("1");
//   }
//   else
//   {
//    object.append("0");
//    }

  //   object = "f215151fsfs5f1515fsfsff";
  sensorstring.data = object.c_str();




  pub.publish(&sensorstring);


  nh.spinOnce();



}

