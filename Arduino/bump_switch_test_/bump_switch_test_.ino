void setup() {
  // put your setup code here, to run once:
DDRG=0X00;
DDRC=0XFF;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(bit_is_clear(PING,0))
  {
    PORTC|=(1<<PC0);
  }
  else 
  PORTC &= ~(1<<PC0);
}
