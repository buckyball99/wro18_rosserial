
#include"avr/io.h"
#include"avr/interrupt.h"


void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:

 EIMSK |= (1 << INT6) ; //VECTORS AND PIN DEFS
      EICRB &= ~(1 << ISC61) ;
          EICRB &= ~(1 << ISC60) ;   
          DDRC=0xFF;
          DDRE &= ~(1<<PE6);
          PORTE|=(1<<PE6);
            sei();
}
  ISR(INT6_vect) {
    Serial.println("1");
    PORTC|=(1<<PC7);
//if(bit_is_clear(PINE,6))
//{
// Serial.println("1"); 
//  PORTC|=(1<<PC7);
//  }
//  PORTC &= ~(1<<PC7);
// }
  }
 

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("as");
 PORTC &= ~(1<<PC7);

}
