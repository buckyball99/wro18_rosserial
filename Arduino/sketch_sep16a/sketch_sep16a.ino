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
#include <std_msgs/Int16.h>

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

#include"avr/io.h"
#include"avr/interrupt.h"

using namespace std;

#define frontSensorCS PF0
#define backSensorCS PF1


ros::NodeHandle nh;
VL53L0X sensor;
float tof_reading;
float sensorbutton_array[3][8];
float sensorsubscribe[24];
int f[8], b[8], w, s, l, r, t, y, e;
std_msgs::String sensorstring;
std_msgs::Int16 button_data;
string object;
const int pingPin = 7; //E7

long duration;
double cm;
int reading_analog;


bool shouldPublish = true;

int frontSensorReading[8];
int previouserrorfront = 0, previouserrorback = 0;

int frontdigital[8], backdigital[8], previousfrontdigital[8], previousbackdigital[8];
int backSensorReading[8], allWhiteFlagback, accurateback = 3500, accuratefront = 3500 , backerror, posback, posfront, fronterror, frontsensorVal[8], backsensorVal[8];
double weightedSumBack, weightedSumFront, sumback, sumfront;



long int averageFrontBlack[8] = {3038 , 3038 , 3038 , 3038  , 2897 , 2897 , 2897,  2897     };
long int averageFrontWhite[8] = {204 , 150, 148, 150, 262, 265, 263, 304 };
long int averageBackBlack[8] = {3051 , 3052 , 3050 , 3050 , 3056 , 3056 , 3056 , 3055 };
long int averageBackWhite[8] = {305 , 159 , 209 , 143 , 135 , 141 , 155 , 155 };
volatile int bumpswitch = 5;




int  frontpwmold, backpwmold;
int digisensorleft, digisensorright, previousdigisensorleft, previousfrontsensorVal[8];
int frontpwmd, backpwmd, leftpwm = 250, rightpwm = 250, allWhiteFlagfront;

unsigned long   weightedSum = 0, sum = 0;
int frontpwm, backpwm;


long sensor_time;

int button_counter = 0;
int button_arr[3] = {0, 0, 0};
bool send_flag = false;

string int_button_array_to_string(int int_button_array[], int size_of_button_array) {
  std::ostringstream os("");
  for (int temp = 0; temp < size_of_button_array; temp++)
    os << int_button_array[temp];
  return os.str();
}

string int_data_to_string(int data) {
  std::ostringstream oss("");
  oss << data;
  return oss.str();
}

void check_input()
{
  if (button_counter < 3)
  {

    if (bit_is_clear(PING, 0))
    {
      button_arr[button_counter] = 1;
      button_counter++;
      while (bit_is_clear(PING, 0));
      _delay_ms(100);
    }

    if (bit_is_clear(PING, 1))
    {
      button_arr[button_counter] = 2;
      button_counter++;
      while (bit_is_clear(PING, 1))
        _delay_ms(100);
    }

    if (bit_is_clear(PING, 2))
    {
      button_arr[button_counter] = 3;
      button_counter++;
      while (bit_is_clear(PING, 2))
        _delay_ms(100);
    }

    if (bit_is_clear(PING, 3))
    {
      button_arr[button_counter] = 4;
      button_counter++;
      while (bit_is_clear(PING, 3))
        _delay_ms(100);
    }

    if (bit_is_clear(PING, 4))
    {
      button_arr[button_counter] = 5;
      button_counter++;
      while (bit_is_clear(PING, 4))
        _delay_ms(100);
    }

    if (bit_is_clear(PINA,1))
    {
      button_arr[button_counter] = 6;
      button_counter++;
      while (bit_is_clear(PINA, 1))
        _delay_ms(100);
    }

    if (bit_is_clear(PINA, 2))
    {
      button_arr[button_counter] = 7;
      button_counter++;
      while (bit_is_clear(PINA, 2))
        _delay_ms(100);
    }

    if (bit_is_clear(PINA, 3))
    {
      button_arr[button_counter] = 8;
      button_counter++;
      while (bit_is_clear(PINA, 3))
        _delay_ms(100);
    }

  }
  else
  {
    send_flag = true;
    button_counter = 0;
  }
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

    frontsensorVal[j] = constrain(map(frontSensorReading[j], averageFrontWhite[j], averageFrontBlack[j], 0, 1000), 0, 1000); // 20  150 1000 0
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


    backsensorVal[j] = constrain(map(backSensorReading[j], averageBackWhite[j], averageBackBlack[j], 0, 1000), 0, 1000); // 20  150 1000 0

    if (backsensorVal[j] > 500)
    {
      backdigital[j] = 1;
    }

    else
      backdigital[j] = 0;

     if(j==7)
  { 
      backsensorVal[j]=0;
      backdigital[j] = 0;
    }
  }
 


  

}

void calculate_pid()

{

  for (int j = 0; j < 8; j++)
  {
    weightedSumFront += (double)(frontsensorVal[j]) * ((j) * 1000);
    // if ( frontdigital[j] == 1)
    //{
    sumfront += frontsensorVal[j];
    //}

  }


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



  frontpwmold = 0 - fronterror;
  backpwmold =  backerror;


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




  frontpwmd = map(frontpwm, 0, 3500, 90, 120);
  backpwmd = map(backpwm, 0, 3500, 90, 120);
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



}

ros::Subscriber<std_msgs::String>sub("AtmegaIn", dataCallback);
ros::Publisher pub("AtmegaOut", &sensorstring);
ros::Publisher button_pub("buttons", &button_data);

void setup()
{
  Serial.begin(57600);

  Wire.begin();
  DDRA = 0X00;
  DDRB = 0xFF;
  //DDRG = 0x00;
  PORTB = 0xFF;
  PORTA |= (1 << PA3) | (1 << PA1) | (1 << PA2) | (1 << PA5);
  PORTG |= (1 << PG0) | (1 << PG1) | (1 << PG2) | (1 << PG3);
  DDRC = 0xFF;
  DDRD |= (1 << PD6) | (1 << PD7);
  PORTD |= (1 << PD6) | (1 << PD7);
  DDRE &= ~(1 << PE6);
  PORTE |= (1 << PE6);
  sensor.init();
  EIMSK |= (1 << INT6);
  EICRB |= (1 << ISC61);//FOR INTERRUPT 2 AND 3(IF CHANGING CHANGE
  EICRB &= ~(1 << ISC60); // falling level

  //VECTORS AND PIN DEFS

  sei();

  sensor.setTimeout(500);
  spiMasterInit();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  nh.advertise(button_pub);

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

  check_input();

  object.append("f");
  for (int i = 0; i < 8; i++) {
    f[i] = frontdigital[i];

  }

  object.append(int_button_array_to_string(f, 8));
  object.append("b");
  for (int i = 0; i < 8; i++) {
    b[i] = backdigital[i];
  }
  object.append(int_button_array_to_string(b, 8));


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
  y = floor(cm * 10);
  object.append(int_data_to_string(y));
  object.append("e");
  object.append(int_data_to_string(bumpswitch));
  object.append("g");

  //  if (bumpswitch == 1)
  //  {
  //    EIMSK &= ~(1 << INT6);
  //    if (bit_is_set(PINE, 6))
  //  {
  //    object.append(int_data_to_string(0));
  //    }
  //    else
  //    {
  //      object.append(int_data_to_string(1));
  //    }
  //  }
  //  else
  //  {
  //    object.append(int_data_to_string(1));
  //    }






  //   object = "f215151fsfs5f1515fsfsff";

  sensorstring.data = object.c_str();

  if (send_flag == true)
  {

    button_data.data = 100*button_arr[0] + 10*button_arr[1] + button_arr[2];


    for (int i = 0; i < 3; i++)
    {
      
      button_arr[i] = 0;
    }
    
    button_pub.publish(&button_data);
    send_flag = false;
    button_data.data = 0;

  }

  

  pub.publish(&sensorstring);


  nh.spinOnce();
  if (bit_is_set(PINE, 6))
  {
    bumpswitch = 1;
  }

}




ISR(INT6_vect)
{
  bumpswitch = 0 ;
}
