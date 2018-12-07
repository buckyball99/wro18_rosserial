#include <pinDefsAutoNew.h>
//                                                              forward and right is front side of bot
#include <io128.h>
#include <sra128.h>
#include "mcp.h"

#define frontSensorCS PF0
#define backSensorCS PF1

int frontSensorReading[8];
int previouserrorfront = 0, previouserrorback = 0;




int frontdigital[8], backdigital[8], previousfrontdigital[8], previousbackdigital[8];
int backSensorReading[8], allWhiteFlagback, accurateback = 3500, accuratefront = 3500 , backerror, posback, posfront, fronterror, frontsensorVal[8], backsensorVal[8];
double weightedSumBack, weightedSumFront, sumback, sumfront;

long int averageFrontBlack[8] = {2754, 2753, 2751, 2750, 2741, 2739, 2738, 2737  };
long int averageFrontWhite[8] = {1366, 1693, 1584, 1212, 846,  260,  1178, 1050};
long int averageBackBlack[8] = {3011,  3013  , 3012  , 3013  , 2991  , 2989  , 2991  , 2989};
long int averageBackWhite[8] = {1415, 1108 , 1066 , 1437 , 1318 , 1032 , 1943, 2140};


int  frontpwmold, backpwmold;
int digisensorleft, previousdigisensorleft, previousfrontsensorVal[8];
int frontpwmd, backpwmd, leftpwm = 250, rightpwm = 250, allWhiteFlagfront;

unsigned long   weightedSum = 0, sum = 0;
int frontpwm, backpwm;

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
    Serial.print(frontsensorVal[k]);
    Serial.print("\t");
    //    Serial.print(frontdigital[k]);
  }
  //  Serial.print();
}

void displayBackValues()
{
  Serial.print("BackSensor : ");
  for (int l = 0 ; l < 8; l++)
  {
    //    Serial.print(backSensorReading[l]);
    //Serial.print("\t");
    Serial.print(backsensorVal[l]);
    Serial.print("\t");

  }
  //Serial.println();
}
void calculatefrontSensorValues()
{
  for (int j = 0; j < 8; j++)
  {

    frontsensorVal[j] = constrain(map(frontSensorReading[j], averageFrontWhite[j], averageFrontBlack[j], 0, 1000), 0, 1000); // 20  150 1000 0
    //    Serial.print(sensorRaw[j]);
    //    Serial.print("  ");
    if (frontSensorReading[j] > 2000 )
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

    if (backSensorReading[j] > 1500)
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
    if (frontSensorReading[j] > 0)
    {
      weightedSumFront += (double)(frontsensorVal[j]) * ((j) * 1000);
      sumfront += frontsensorVal[j];
    }
  }

  // Serial.print(weightedSumFront);
  // Serial.print("\t");
  // Serial.println(sumfront);

  if (sumfront != 0)
  {

    posfront = (double)weightedSumFront / sumfront ;
  }
  //  Serial.print("\t");
  //  Serial.print("\t");
  //  Serial.print("\t");

  // Serial.println(posfront);
  for (int j = 0; j < 8; j++)
  {

    if (backSensorReading[j] > 0)
    {
      weightedSumBack += (double)(backsensorVal[j]) * ((j) * 1000);
      sumback += backsensorVal[j];
    }
  }

  if (sumback != 0)
  {
    posback = (double)weightedSumBack / sumback;
  }
  // Serial.print("\t");
  //Serial.println(posback);
  //    Serial.print("\t");
  //   Serial.print("\t");

  // Serial.println(weightedSumBack);

  fronterror = posfront - accuratefront;
  backerror = posback - accurateback;

  // Serial.print(fronterror);
  //  Serial.print("\t");
  //  Serial.println(backerror);



  frontpwmold = 0 - fronterror;
  backpwmold = backerror;

  //  Serial.print(frontpwmold);
  //  Serial.print("\t");
  //  Serial.println(backpwmold);


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
      frontpwmold = 3500;                // changed it posi and nega
    else if (previousfrontdigital[7] == 1)
      frontpwmold = -3500;
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
    if (previousbackdigital[0] == 1 )               // changed it posi and nega
      backpwmold = -3500;
    else if (previousbackdigital[7] == 1)
      backpwmold = 3500;
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
    MOTORBF = 0; // FRONTT   CHANGE DOR OF MOTOR
    MOTORBB = 1;
  }


  frontpwmd = map(frontpwm, 250, 3500, 80, 200);
  backpwmd = map(backpwm, 250, 3500, 80, 200);
  if (frontpwm< 250)
  {
    frontpwmd = 0;
  }
  if (backpwm < 250)
  { backpwmd = 0;
  }
  weightedSumFront = 0; weightedSumBack = 0; sumfront = 0; sumback = 0;
  previouserrorfront = fronterror;
  previouserrorback = backerror;
  for (int i; i < 8; i++)
  {
    backsensorVal[i];
    frontsensorVal[i];
    //    Serial.println();
  }
  //  Serial.println();
  //  Serial.print(posfront);
  //  Serial.print("\t");
  //  Serial.print(posback);
  //  Serial.print("\t");

  //  Serial.print(frontpwmd);
  //  Serial.print("\t");
  //  Serial.print(backpwmd);
  //  Serial.print("\t");
  //  Serial.println();

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
int main() {
  //  PORTA |= (1 << PA4);

  //  if(bit_is_clear)
  // while (bit_is_set(PINA, 4));
  //{
  Serial.begin(115200);
  DDRC = 0xFF;
  spiMasterInit();
  //  Serial.println("h");
  DDRD |=  (1 << PD6) | (1 << PD7);
  PORTD |=   (1 << PD6) | (1 << PD7);
  // put your setup code here, to run o
  pwm1_init();
  pwm3_init();

  while (1)
  {
    readBackSensor();
    readFrontSensor();
    calculatefrontSensorValues();
    displayFrontValues();
    moveonline();
    Serial.print(frontpwmold);
    Serial.print("\t");
    Serial.print(frontpwmd);
    Serial.print("\t");

    calculatebackSensorValues();
    displayBackValues();
    Serial.print(backpwmold);
    Serial.print("\t");
    Serial.print(backpwmd);
    Serial.print("\t");
    Serial.println();







  }
}




void BOT_FORWARD()
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;
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
  MOTORRF = 0;// RIGHT
  MOTORRB = 1;
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
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;

  MOTORBF = 1;// BACK
  MOTORBB = 0;

  MOTORLF = 1; // LEFTb b
  MOTORLB = 0;

  MOTORFF = 0; // FRONTT
  MOTORFB = 1;
}
void SPOT_RIGHT()               // change later
{
  MOTORRF = 0;// RIGHT
  MOTORRB = 1;

  MOTORBF = 0;// BACK
  MOTORBB = 1;

  MOTORLF = 0; // LEFT
  MOTORLB = 1;

  MOTORFF = 1; // FRONTT
  MOTORFB = 0;

}

void BOT_STOP()
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 1;

  MOTORBF = 1;// BACK
  MOTORBB = 1;

  MOTORLF = 1; // LEFT
  MOTORLB = 1;

  MOTORFF = 1; // FRONTT
  MOTORFB = 1;
}

