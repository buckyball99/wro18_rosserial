#include <IO16.h>
#include <SRA16.h>
//
//#include <io128.h>
//#include <sra128.h>

// 1. agar pehla led (B0) jala toh dist 1200 se kum he which is perfect
// 2. agar dusra led (B1) jala toh dist 1200 se jyada he which means data aaya but sahi nahi he coz usko 3.3 logic nahi samjha
// 3. agar dono led jala toh dist calculate hi nahi ho raha
// 4. agar koi led nahi jala toh code nahi chala
// 5. 2nd and 3rd condition me logic level convertor use karke dekho.
// har loop ke liye led line se jalaya he, code dekho samjhega

/*For Arduino board with multiple serial ports such as DUE board, comment out the above two codes, and directly use Serial2 port*/

#include<avr/eeprom.h>
#include<avr/io.h>
uint16_t dist= 0,a=0;// LiDAR actually measured distance value
uint16_t check;// check numerical value storage
int i;
byte uart[9];// store data measured by LiDAR
byte HEADER=0x59;// data package frame header (check datasheet)

void setup() 
{
 DDRB =0b00000011;
 DDRC =0b11111100;
 Serial.begin(115200);
 PORTB = 0b00000000;
 PORTC = 0b00000000;
}

void loop() 
{
  PORTC = 0b00000000;
  if (Serial.available())//check whether the serial port has data input
  {
            PORTC|=0b00000100;
            delay(1); // this delay stabilizes code
            uart[0] = Serial.read();
            
    if(uart[0]==HEADER)// determine data package frame header 0x59
    {
              PORTC|=0b00001000;
              uart[1] = Serial.read();
              
      if(uart[1]==HEADER)//determine data package frame header 0x59
      {
        PORTC|=0b00010000;
        for(i=2;i<9;i++)// store data to array
        {
          uart[i]=Serial.read();
        }
      check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
      
      if(uart[8]==(check&0xff))// check the received data as per protocols
      {
        PORTC|=0b00100000;
        dist = 0;
        dist = dist | uart[3];// higher bits
        dist = dist<<8;
        dist = dist | uart[2];// lower bits
Serial.println(dist);
//        if(dist<=1200)
//          PORTB = 0b00000001;
//        else if(dist>1200)
//          PORTB = 0b00000010;
//        else
//          PORTB = 0b00000011;
       }
     }
   }
   
   
 }


}
