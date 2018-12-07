void setup() {
  // put your setup code here, to run once:
DDRA=0X00;
Serial.begin(9600);
PORTA=0X00;}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println(PA4);
}   
