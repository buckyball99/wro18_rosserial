#include <pinDefsAutoNew.h>
//                                                                      forward and right is front side of bot
#include <io128.h>
#include <sra128.h>
#include "mcp.h"
#include <Wire.h>
#include <VL53L0X.h>


#define frontSensorCS PF0
#define backSensorCS PF1
VL53L0X sensor;
int frontSensorReading[8];

#define MAX_PWM 200
#define MIN_PWM 20
//int pwmthreshold = 9;
int bot_said_left = 'y';
int digiCounter = 0;
char condition = 'a';
int Jcounter = 0;
int leftFlag=1;
int backSensorReading[8], allWhiteFlagback, digiflag, sumback, accurateback = 35, accuratefront = 35, backerror, posback, sumfront, posfront, fronterror, frontsensorVal[8], backsensorVal[8];
long int weightedSumBack, weightedSumFront;
int frontjunction = 0, count = 0;
int previousdigisensorleft = 0;
int  frontpwmold, backpwmold, flag, countflag = 0;
int digisensorleft;
int frontpwmd, backpwmd, leftpwm = 150, rightpwm = 150, allWhiteFlagfront;
int readVal, positionVal, accurateval = 29, accuratevalback = 25, previouserror1 , junctionno , yawerror1, yawerror2 = 0, lateralerror1 = 0, lateralerror2 = 0, totalerror1 = 0, totalerror2 = 0;
//int thresh = 50;
float kp = 1, ki = 0, kd = 0, p, error1,   yawcorrection1;
// Variables to store analog and line position value
//int junction,  differencialerror2, cumulativeerror2;
bool stopFlag = 0;
//const byte analogPin = 61;   // Connect AN output of LSA08 to analog pin 0 F0 IS 61  TO F7 IS 54  //
//const byte junctionPulse = 25;                                        // D0 IS 25 PIN                        //
int junctioncount = 0;
int sensorRaw[8], sensorVal[8];
int allWhiteFlag;
unsigned long   weightedSum = 0, sum = 0;
int a, posonwhite = 3000 , error, pos ;
int posaccurate = 25, olderror, srasensorerror;
int reading_analog, reading_digital;

int previouserror, frontpwm, backpwm;

void readFrontSensor()
{
  for (int i = 0; i < 8; i++)
  {
    frontSensorReading[i] = getSensorReading(frontSensorCS, i);

  }
}
void readBackSensor()
{
  for (int j = 0; j < 8; j++)
  {
    backSensorReading[j] = getSensorReading(backSensorCS, j);

  }
}
void displayFrontValues()
{
  Serial.print("FrontSensor : ");
  for (int k = 0 ; k < 8; k++)
  {
    Serial.print(frontSensorReading[k]);
    Serial.print("\t");
    Serial.print(frontsensorVal[k]);
    Serial.print("\t");
  }
  Serial.println();
} void displayBackValues()
{
  Serial.print("BackSensor : ");
  for (int l = 0 ; l < 8; l++)
  {
    Serial.print(backSensorReading[l]);
    Serial.print("\t");
    Serial.print(backsensorVal[l]);
    Serial.print("\t");

  }
  Serial.println();
}
void calculatefrontSensorValues()
{
  for (int j = 0; j < 8; j++)
  {
    //    sensorVal[j] = map(sensorRaw[j], 0, 700, 0, 2000);    // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");

    if (frontSensorReading[j] <= 2200)
      frontsensorVal[j] = 0;
    else if (frontSensorReading[j] > 2200)
      frontsensorVal[j] = 1;


    //    Serial.print(frontsensorVal[j]);
    //    Serial.print("\t");

    //                Serial.print(previousfrontsensorVal[j]);
    //                Serial.println("\t");
  }


}
void calculatebackSensorValues()
{
  for (int j = 0; j < 8; j++)
  {
    //    sensorVal[j] = map(sensorRaw[j], 0, 700, 0, 2000);    // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");

    if (backSensorReading[j] <= 2200)
      backsensorVal[j] = 0;
    else if (backSensorReading[j] > 2200)
      backsensorVal[j] = 1;
  }
}
void calculate_pid()
{
  for (int j = 0; j < 8; j++)
  {
    weightedSumFront += (long)(frontsensorVal[j]) * ((j) * 10);
    sumfront += frontsensorVal[j];
  }
  posfront = weightedSumFront / sumfront;
  //  Serial.println(posfront);
  for (int j = 0; j < 8; j++)
  {
    weightedSumBack += (long)(backsensorVal[j]) * ((j) * 10);
    sumback += backsensorVal[j];
  }
  posback = weightedSumBack / sumback;
  //  Serial.println(posback);
  fronterror = posfront - accuratefront;
  backerror = posback - accurateback;

  frontpwmold = fronterror;
  backpwmold = 0 - backerror;
  if (fronterror < -35)
  {
    allWhiteFlagfront = 1;
  }
  if (    allWhiteFlagfront == 1)

  {
    if (previouserror1 >= 0 )
      frontpwmold = 35;
    else if (previouserror < 0)
      frontpwmold = -35;
  }
  fronterror = posfront - accuratefront;
  allWhiteFlagfront = 0;

  if (backerror > 35)
  {
    allWhiteFlagback = 1;
  }
  if (    allWhiteFlagback == 1)

  {
    if (previouserror1 >= 0 )
      backpwmold = 35;
    else if (previouserror < 0)
      backpwmold = -35;
  }
  fronterror = posfront - accuratefront;
  backerror = posback - accurateback;

  allWhiteFlagfront = 0;
  allWhiteFlagback = 0;
  //  Serial.print(frontpwm);
  //    Serial.print(\t);
  //    Serial.print(backpwm);
  //    Serial.println();
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
    MOTORBF = 0; // FRONTT   CHANGE DOR OF MOTOR
    MOTORBB = 1;
  }
  //  Serial.print(frontpwmold);
  //    Serial.print("\t");
  //    Serial.print(backpwmold);
  //    Serial.println();
  frontpwmd = map(frontpwm, 0, 35, 80, 120);
  backpwmd = map(backpwm, 0, 35, 80, 120);
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
void set_pwm()
{
  PWMF = frontpwmd;
  PWMB = backpwmd;
  PWML = leftpwm;
  PWMR = rightpwm;
}
void set_pwm_new(int f, int b, int l, int r)
{
  PWMF = f;
  PWMB = b;
  PWML = l;
  PWMR = r;
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

int front_sensor_J()
{
  if ( frontsensorVal[2] == 1 || frontsensorVal[0] == 1 ||frontsensorVal[1] == 1 )
  {
    return 1;
  }

  else
  {
    return 0;
  }
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
void moveonline()
{
  calculate_pid();
  set_pwm();
  BOT_FORWARD();
}
void readdigisensorleft()
{
  if (bit_is_set(PIND, 3))
  {
    digisensorleft = 1;

  }
  else
  {
    digisensorleft = 0;
  }
}
void Tof_detection(){
  
  reading_analog= sensor.readRangeContinuousMillimeters(); 
  Serial.print("Reading: /t");
  Serial.println(reading_analog);
  }

 void Tof_convert(){
  if(reading_analog >= 20 && reading_analog <=200 )
     reading_digital=1;

   else
      reading_digital=0;
  
  }



int main() {
  //  PORTA |= (1 << PA4);

  //  if(bit_is_clear)
  // while (bit_is_set(PINA, 4));
  //{
  Serial.begin(115200);
  DDRC = 0xFF;
  spiMasterInit();
  //    Serial.println("h");
  DDRD |=  (1 << PD6) | (1 << PD7);
  PORTD |=   (1 << PD6) | (1 << PD7);
  // put your setup code here, to run o
  pwm1_init();
  pwm3_init();

  while (1)
  {
    Serial.print("digisensor:");
    Serial.print(digisensorleft);
    Serial.print("\t");
    Serial.print("previousdigisensor:");
    Serial.print(previousdigisensorleft);
    Serial.print("\t");
    Serial.print("DigiValue:");
    Serial.println(digiCounter);

    readBackSensor();
    readFrontSensor();
    calculatefrontSensorValues();
    calculatebackSensorValues();
    
    Tof_detection();
    Tof_convert();
    //      displayFrontValues();
    readdigisensorleft();

    if (digisensorleft == 1 && previousdigisensorleft == 0)
    {
      digiCounter++;

    }

    previousdigisensorleft = digisensorleft;

    if (digiCounter < 2 )
    {
      Serial.println("hi how araee you");
      BOT_FORWARD();
      set_pwm_new(0, 0, 250, 250);
    }
    else if ((digiCounter == 2 || digiCounter == 3 ) && leftFlag==1)
    {

      if (front_sensor_J() == 0)
      {

        Serial.println("a");
        SPOT_LEFT();
        set_pwm_new(100, 100, 100, 100);
      }

      else
      {
        Serial.println("b");
        BOT_STOP();
        leftFlag=0;
      }
    }
    if (digiCounter == 3 && leftFlag==0)
    {
      moveonline();
    }
    if(digiCounter == 4)
    {
      SPOT_LEFT();
      set_pwm_new(100,100,100,100);
      }
    if(reading_digital==0)
    {
      moveonline();
      
    }
    else
    {
      BOT_STOP();
    }
      
  }

   


  Serial.println("");


  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;

  // previousfrontsensorVal[i]=frontsensorVal[i];

}



