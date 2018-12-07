#include <pinDefsAutoNew.h>
//                                                              forward and right is front side of bot
#include <io128.h>
#include <sra128.h>
#include <mcp.h>
#include <Wire.h>
#include <VL53L0X.h>

#define frontSensorCS PF0
#define backSensorCS PF1

int frontSensorReading[8];
int previouserrorfront = 0, previouserrorback = 0;
VL53L0X sensor;
#define MAX_PWM 200
#define MIN_PWM 20



//int pwmthreshold = 9;

int bot_said_left = 'y';
char condition = 'a';
int digiCounter = 0;
int leftFlag = 1;
int r = 0, t = 0;
int frontdigital[8], backdigital[8], previousfrontdigital[8], previousbackdigital[8];
int backSensorReading[8], allWhiteFlagback, accurateback = 41, accuratefront = 32 , backerror, posback, posfront, fronterror, frontsensorVal[8], backsensorVal[8];
double weightedSumBack, weightedSumFront, sumback, sumfront;
int frontjunction = 0, count;
int  frontpwmold, backpwmold;
int digisensor, previousdigisensor, digisensorleft, previousdigisensorleft, previousfrontsensorVal[8];
int frontpwmd, backpwmd, leftpwm = 150, rightpwm = 150, allWhiteFlagfront;
int readVal, positionVal, accurateval = 29, accuratevalback = 25, previouserror1 , junctionno , yawerror1, yawerror2 = 0, lateralerror1 = 0, lateralerror2 = 0, totalerror1 = 0, totalerror2 = 0;
//int thresh = 50;
int reading_analog, reading_digital;


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
    //    Serial.print(frontSensorReading[k]);
    //    Serial.print("\t");
    //    Serial.print(frontsensorVal[k]);
    //    Serial.print("\t");
    Serial.print(frontdigital[k]);
  }
  Serial.println();
}

void displayBackValues()
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

    frontsensorVal[j] = constrain(map(frontSensorReading[j], 100, 3000, 0, 10000), 0, 10000); // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");
    if (frontsensorVal[j] > 9000 )
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

    if (backsensorVal[j] > 9000)
    {
      backdigital[j] = 1;
    }

    else
      backdigital[j] = 0;


  }

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

void Tof_detection() {

  reading_analog = sensor.readRangeContinuousMillimeters();
  Serial.print("Reading: \t");
  Serial.println(reading_analog);
}



void Tof_convert() {
  if (reading_analog >= 20 && reading_analog <= 200 )
    reading_digital = 1;

  else
    reading_digital = 0;

}


void calculate_pid()
{
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
  //  Serial.print("\t");
  //  Serial.println(posback);
  fronterror = posfront - accuratefront;
  backerror = posback - accurateback;

  /*  Serial.print(fronterror);
    Serial.print("\t");
    Serial.println(backerror);*/



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
    for (int f = 0; f < 8; f++)
    {
      previousfrontdigital[f] = frontdigital[f];
    }
  }


  if (allWhiteFlagfront == 1)

  {
    if (previousfrontdigital[0] == 1)
      frontpwmold = -22;                // changed it posi and nega
    else if (previousfrontdigital[7] == 1)
      frontpwmold = 22;
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
    if (previousbackdigital[0] == 1 )
      backpwmold = 26;
    else if (previousbackdigital[7] == 1)
      backpwmold = -26;
  }

  //  fronterror = posfront - accuratefront;
  //  backerror = posback - accurateback;

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


  frontpwmd = map(frontpwm, 0, 22, 60, 150);
  backpwmd = map(backpwm, 0, 26, 60, 150);
  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;
  previouserrorfront = fronterror;
  previouserrorback = backerror;
  /*     Serial.print(frontpwmd);
        Serial.print("\t");
        Serial.print(backpwmd);
        Serial.println();*/
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
void moveonline()
{
  calculate_pid();
  set_pwm();
  BOT_FORWARD();
}
/*void readdigisensor()
  {
  if (bit_is_set(PIND, 0))
  {
    digisensor = 1;
  }
  else
  {
    digisensor = 0;
  }
  }*/

int front_sensor_J()
{
  if (frontdigital[6] == 1 || frontdigital[5] == 1)
  {
    return 1;
  }

  else
  {
    return 0;
  }
}

int front_sensorss_J()
{
  if (frontdigital[5] == 1 || frontdigital[6] == 1 || frontdigital[2] == 1 || frontdigital[3] == 1 || frontdigital[4] == 1 )
  {
    return 1;
  }

  else
  {
    return 0;
  }
}

void init_tof()
{

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();


}

void setup()
{
  //  PORTA |= (1 << PA4);

  //  if(bit_is_clear)
  // while (bit_is_set(PINA, 4));
  //{
  Serial.begin(115200);


  DDRC = 0xFF;
  spiMasterInit();
  Wire.begin();
  init_tof();

  //  Serial.println("h");
  DDRD |=  (1 << PD6) | (1 << PD7);
  PORTD |=   (1 << PD6) | (1 << PD7);
  // put your setup code here, to run o


  pwm1_init();
  pwm3_init();


  //PORTC|=0xFF;
}
void loop()
{
  Tof_detection();
  Tof_convert();
  //PORTC|=0xFF;
  readBackSensor();
  readFrontSensor();
  calculatefrontSensorValues();
  //displayFrontValues();
  calculatebackSensorValues();
  //Serial.print("A");
  readdigisensorleft();
  //moveonline();

  Serial.println(digiCounter);

  if (digisensorleft == 1 && previousdigisensorleft == 0)
  {
    digiCounter++;

  }

  previousdigisensorleft = digisensorleft;

  if (digiCounter < 2 )
  {
    //      Serial.println("hi how araee you");
    BOT_FORWARD();
    set_pwm_new(0, 0, 150, 150);
  }


  else if ((digiCounter == 2 ) && leftFlag == 1)
  {
    if (r == 0)
    {

      BOT_STOP();
      _delay_ms(1000);
      r = 1;
    }

    if (front_sensor_J() == 0 && r == 1)
    {

      //Serial.println("a");
      SPOT_LEFT();
      set_pwm_new(100, 100, 100, 100);
    }

    else
    {

      BOT_STOP();
      _delay_ms(1000);
      leftFlag = 0;
    }


    if ((digiCounter == 2 || digiCounter == 3 ||  digiCounter == 4 )  && leftFlag == 0 && reading_digital == 0)
    {
      //Serial.println("b");
      moveonline();
    }
    else
    {
     BOT_STOP();
      _delay_ms(1000);
      // pick  up the block 
      }
   /* if (t == 0  && digiCounter == 3)
    {
      BOT_STOP();
      _delay_ms(1000);
      t = 1;
    }
    if ( t == 1 && front_sensorss_J() == 1)
    {
      SPOT_LEFT();
      set_pwm_new(100, 100, 100, 100);
      t = 2;
    }
    if ( t == 2 && front_sensorss_J() == 0)
    {
      SPOT_LEFT();
      set_pwm_new(100, 100, 100, 100);
      t = 3;
    }

    //    if (front_sensor_J() == 0 && t==2 )
    //    {
    //      SPOT_LEFT();
    //      set_pwm_new(100, 100, 100, 100);
    //      u=1;
    //    }

    else if (digiCounter == 4 && t == 3 && front_sensor_J() == 1)
    {

      BOT_STOP();
      _delay_ms(1000);
      t = 4;
    }

    if (t == 4)
    {
      moveonline();

      if (reading_digital == 1)
      {
        Serial.print("F");
        BOT_STOP();
        _delay_ms(1000);
      }
    }
    */









  }
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
