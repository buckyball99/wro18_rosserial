#include<avr/io.h>
#include<stdlib.h>
#include<compat/deprecated.h>
#include<util/delay.h>
#include<avr/eeprom.h>
#include<IO16.h>
#include <inttypes.h>


void adc_init(void)
{
 ADC_DIR=0X00;
 ADCSRA=0X00;
 ADMUX=0X40;//0x40 for 10 bits
 ADCSRA=0X87;
 ACSR=0X80;
}

//ADC START
unsigned char adc_start(unsigned char channel)
{
 unsigned char i;
 
     ADCH=0x00;

   i=channel&0x07;
   ADMUX=i|0x40;        //i|0x40 for 10 bits
   ADCSRA|=1<<ADSC;
     
     while(ADCSRA & (1<<ADSC));       // wait for conv. to complete
     unsigned char temp=ADCH;      //unsigned int temp=ADC;   for 10 bits
   
 return temp;
}

void setup()
{
  Serial.begin(115200);   
  adc_init();
}
 
void loop()
{
  int val = 13*pow((adc_start(0)*0.0048828125),-1);       // reads the value of the sharp sensor
  Serial.println(val);            // prints the value of the sensor to the serial monitor
  //delay(400);                    // wait for this much time before printing next value
}
