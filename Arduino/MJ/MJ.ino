void setup() {
  // put your setup code here, to run once:
DDRA=0x00;
PORTA=(1<<PA4);
}

void loop() {
  // put your main code here, to run repeatedly:
if(bit_is_set(PINA,5))
PORTC=0XFF;
else
PORTC=0X00;
}
