//Base Motors #define Start
#define MOTORFF    REGISTER_BIT(PORTC,0)
#define MOTORFB    REGISTER_BIT(PORTC,1)
#define PWMF       OCR1A

#define MOTORLF    REGISTER_BIT(PORTC,2)
#define MOTORLB    REGISTER_BIT(PORTC,3)
#define PWML       OCR1B

#define MOTORBF    REGISTER_BIT(PORTC,5)
#define MOTORBB    REGISTER_BIT(PORTC,4)
#define PWMB       OCR3A

#define MOTORRF    REGISTER_BIT(PORTC,7)
#define MOTORRB    REGISTER_BIT(PORTC,6)
#define PWMR       OCR3B

#define BASEBRAKE  REGISTER_BIT(PORTD,7)
#define BASEBRAKE_DIR  REGISTER_BIT(DDRD,7)
//Base Motors #define End

//Sensorboard #define Start
#define MOSI PB2
#define MISO PB3
#define SCLK PB1
#define SS PB0

#define rightSensor   1 	//PF1
#define leftSensor    2		//PF2

#define frontSensor   0		//PF0
#define backSensor    3		//PF3
//Sensorboard #define End

//Clamp Piston #define Start
#define OPENCLAMP    	REGISTER_BIT(PORTA,2)		//E2
#define OPENCLAMP_DIR   REGISTER_BIT(DDRA,2)
//Clamp Piston #defines End

//Throwing Motors #define Start
#define MOTORFR			REGISTER_BIT(PORTD,3)
#define MOTORBR    		REGISTER_BIT(PORTD,2)
#define PWMT	   		OCR1C
#define MOTORFR_DIR     REGISTER_BIT(DDRD,3)
#define MOTORBR_DIR    	REGISTER_BIT(DDRD,2)

#define THROWBRAKE  	REGISTER_BIT(PORTD,6)
#define THROWBRAKE_DIR  REGISTER_BIT(DDRD,6)
//Throwing Motors #define End

//PI Interfacing #define Start
#define TZ1    	   REGISTER_BIT(PINF,5)
#define TZ2    	   REGISTER_BIT(PINF,4)
#define TZ3    	   REGISTER_BIT(PINF,6)
#define TOF    	   REGISTER_BIT(PINF,7)

#define TZ1_PORT   REGISTER_BIT(PORTF,5) 
#define TZ2_PORT   REGISTER_BIT(PORTF,4) 
#define TZ3_PORT   REGISTER_BIT(PORTF,6)
#define TOF_PORT   REGISTER_BIT(PORTF,7)

#define TZ1_DIR    REGISTER_BIT(DDRF,5) 
#define TZ2_DIR    REGISTER_BIT(DDRF,4) 
#define TZ3_DIR    REGISTER_BIT(DDRF,6)
#define TOF_DIR    REGISTER_BIT(DDRF,7)
//PI Interfacing #define End
