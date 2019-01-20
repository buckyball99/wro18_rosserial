#include <pinDefsAutoNew.h>
//                                                                      forward and right is front side of bot
#include <io128.h>
#include <sra128.h>
#define MAX_PWM 636
#define MIN_PWM 100
int readVal, positionVal, accurateval = 33, previouserror = 0, junctionno ;
int thresh = 50;
float kp = 6, ki = 0, kd = 0, p, differencialerror, cumulativeerror, error, speeed = 270, yawcorrection;
// Variables to store analog and line position value
int junction,leftpwm, rightpwm;
bool stopFlag = 0;
const byte analogPin = 61;   // Connect AN output of LSA08 to analog pin 0 F0 IS 61  TO F7 IS 54  //
const byte junctionPulse = 25;                                        // D0 IS 25 PIN                        //

void calculate_pid()
{
  readVal = adc_start(0);
  junction = adc_start(1);
  junctionno = (((float)junction / 921) * 70);
  positionVal = (((float)readVal / 921) * 70);

  error = positionVal - accurateval;
  p = error;
  if (error > 44) {

    if (error > 44  && rightpwm > leftpwm)
    {
      leftpwm = MIN_PWM;
      rightpwm = MAX_PWM;
    }
    if (error > 44 &&  rightpwm < leftpwm)        // as ot goes t right then the pwwm pins dir mustbechangd
    {
      leftpwm = MAX_PWM;
      rightpwm = MIN_PWM;
    }
  }
  else
  {

    differencialerror = error - previouserror;
    previouserror = error;

    cumulativeerror += error;
    if (cumulativeerror > thresh)
      cumulativeerror = thresh;
    else if (cumulativeerror < (-1)*thresh)
      cumulativeerror = (-1) * thresh;


    yawcorrection = p * kp + differencialerror * kd + cumulativeerror * ki;

    leftpwm = speeed + yawcorrection;
    rightpwm = speeed - yawcorrection;



    leftpwm = constrain(leftpwm, MIN_PWM, MAX_PWM);
    rightpwm = constrain(rightpwm, MIN_PWM, MAX_PWM);
  }
  if (junction==1)
  {
    stopFlag = 1;
  }
  if (stopFlag == 1) { //Serial.println("1");   
   BOT_STOP();
   // stopFlag = 0;
  }
  else BOT_FORWARD();


}




void BOT_BACKWARD()
{
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;


  MOTORLF = 1; // LEFT
  MOTORLB = 0;

}

void set_pwm()
{
  PWMF = leftpwm;
  PWMB = rightpwm;
 PWML = leftpwm;
  PWMR = rightpwm;
}

void BOT_FORWARD()
{
  MOTORLF = 1; // LEFT
  MOTORLB = 0;
  
  MOTORRF = 1;// RIGHT
  MOTORRB = 0;
//
//
//  //
//  MOTORBF = 0;// BACK
//  MOTORBB = 1;
//
//  //
//  MOTORFF = 0; // FRONTT
//  MOTORFB = 1;

}

void SPOT_RIGHT()
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


void SPOT_LEFT()
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

int main() {
  Serial.begin(115200); 
 
  DDRC = 0XFF;
  DDRD |= (1 << PD6) | (1 << PD7);
  PORTD |= (1 << PD6) | (1 << PD7);
  // put your setup code here, to run o
  pwm1_init();
  pwm3_init();
 // pinMode(junctionPulse, OUTPUT);

  adc_init();
  



while(1) {
  // put your main code here, to run repeatedly:

 
  calculate_pid();
  set_pwm();
  if(stopFlag==1)
  {
    Serial.println("0");
    stopFlag=0;
  }
  else {
    BOT_FORWARD();
    }
  
   Serial.print(positionVal);
   Serial.print ("      ");
   Serial.println (junctionno);

   
//   if ()
//    if (digitalRead(junctionPulse))
//  {
//    stopFlag = 1;
//  }
//  if (stopFlag == 1) { //Serial.println("1");   //Bot_stop();
//    stopFlag = 0;
//  }
//  else Bot_Forward();


}
}
