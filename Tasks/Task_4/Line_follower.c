#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include<stdio.h>
#include<stdlib.h>
#include <math.h>
#include "uart.h"
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char L = 0;
unsigned char C = 0;
unsigned char R = 0;
 char R_v[10];
 char L_v[10];
char strC[10];
char strR[10];
char strL[10];
	int count=0;

#define max_tym  50 

void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
void servo4_pin_config (void)
{
	DDRE  = DDRE | 0x08;  //making PORTB 7 pin output
	PORTE = PORTE | 0x08; //setting PORTB 7 pin to logic 1
}
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
void servo_port_init(void)
{
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
	servo4_pin_config();
}



void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void port_init()
{

	adc_pin_config();
	motion_pin_config();
	servo_port_init();         //servo instlln
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin	
}
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
void timer3_init(void)
{
 TCCR3B = 0x00; //stop
 TCNT3H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT3L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR3AH = 0x03;	//Output compare Register high value for servo 1
 OCR3AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR3BH = 0x03;	//Output compare Register high value for servo 2
 OCR3BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR3CH = 0x03;	//Output compare Register high value for servo 3
 OCR3CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR3H  = 0x03;	
 ICR3L  = 0xFF;
 TCCR3A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR3C = 0x00;
 TCCR3B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;



	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);
}

void soft_right (void)
{
	motion_set (0x04);
}

void soft_left (void)
{
	motion_set (0x02);
}

void right (void)
{
	motion_set (0x05);
}

void left (void)
{
	motion_set (0x0A);
}

void backward (void)
{
	motion_set (0x09);
}

void right_back(void)
{
	motion_set(0x08);
	
}

void left_back(void)
{
	motion_set(0x01);
	
}
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
  motion_set (0x00);
}
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 0.35; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM /0.35; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	backward();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
void right_turn_for(){
	 stop();
	 _delay_ms(250);
	forward_mm(70);
	left();
	angle_rotate(160);

	}

void left_turn_for(){
	 stop();
	 _delay_ms(2250);
	forward_mm(70);
	right();
	angle_rotate(160);

}
void right_turn(){
	stop();
	_delay_ms(250);
	
	left();
	angle_rotate(160);

}

void left_turn(){
	stop();
	_delay_ms(250);
	
	right();
	angle_rotate(160);

}

void servo_1(unsigned char degrees)        // servo fns starts 
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}
void servo_4(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR3AH = 0x00;
	OCR3AL = (unsigned char) PositionServo;
}


//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}
void servo_4_free (void) //makes servo 3 free rotating
{
	OCR3AH = 0x03;
	OCR3AL = 0xFF; //Servo 3 off
}
void servo2attack()
{
	int i;
	for(i=100;i<=140;i++)
	{
		servo_2(i);
		_delay_ms(15);
	}
}

void servo2straight()
{
	int i;
	for(i=140;i>=100;i--)
	{
		servo_2(i);
		_delay_ms(15);
	}
}
void servo3straight()
{
	int i;
	for(i=20;i<=150;i++)
	{
		servo_3(i);             // third servo
		_delay_ms(15);
	}
}
void servo3attack()
{
	int i;
	for(i=150;i>=20;i--)
	{
		servo_3(i);             // third servo // port b 6
		_delay_ms(15);
	}
}
void servo1straight()
{
	int i;
	for(i=125;i<=180;i++)
	{
		servo_1(i);             // first servo
		_delay_ms(15);
	}
}
void servo1attack()
{
	int i;
	for(i=180;i>=125;i--)
	{
		servo_1(i);             // first servo
		_delay_ms(20);
	}
}
void servo2backgo()
{
	int i;
	for(i=100;i<=210;i++)
	{
		servo_2(i);             // first servo
		_delay_ms(20);
	}
	
}
void servo2backstraight()
{
	int i;
	for(i=210;i>=100;i--)
	{
		servo_2(i);             // first servo
		_delay_ms(20);
	}
	
}
void downposn()
{
	servo_2(160);
	_delay_ms(100);
	servo3attack();
}


void straight_position()
{
	servo_1(180);
	_delay_ms(250);
	
	servo_3(150);
	_delay_ms(250);
	servo_4(90);
	_delay_ms(250);
	servo_2(100);
	_delay_ms(250);
	
	
}

void pick(){
	servo2backstraight();
	servo_1(135);
	_delay_ms(100);
	servo_2(100);
	int i;
	for(i=150;i>=60;i--)
	{
		servo_3(i);             // first servo
		_delay_ms(15);
	}
	servo_1(180);
	_delay_ms(250);
	for(i=60;i<=150;i++)
	{
		servo_3(i);
		_delay_ms(15);
	}
	straight_position();
}

void placebigh()
{
	servo2backstraight();
	int i;
	for(i=100;i>=0;i--)
	{
		servo_2(i);
		_delay_ms(15);
	}
	_delay_ms(250);
	servo_1(180);
	
}                                             // servo functions ends
void sweep(void){                              //line follower fns starts 
	forward();
	velocity(80,80);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	
	if(R < 50 && C < 50 && L < 50){
		uart0_puts("calibrating sweep\n");
		forward();
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		
		while(R < 50 && C < 50 && L < 50){

			uart0_puts("Sweep init\n");
			int i = 0;
			while((R < 50 && C < 50 && L < 50) && ( 10 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				angle_rotate(5);
				velocity(175,200);
// 				velocity(175,200);
//      			_delay_ms(2);
				i++;
// 				itoa(i,L_v,10);
// 				uart0_puts(L_v);
// 				uart0_puts("\n");
				
			}
			if(R < 50 && C < 50 && L < 50){
				;
			}
			else{
				break;
			}
			int j  = 0;
			while((R < 50 && C < 50 && L < 50) && ( 10 > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				velocity(175,200);
				uart0_puts("Sweep right\n");
				angle_rotate(5);
// 				
// 				_delay_ms(2);
				j++;
// 				itoa(j,R_v,10);
// 				uart0_puts(R_v);
// 				uart0_puts("\n");
			}
			if(R < 50 && C < 50 && L < 50){
				;
			}
			else{
				break;
			}
			j  = 0;
			while((R < 50 && C < 50 && L < 50) && ( 10 > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				velocity(175,200);
				angle_rotate(5);
// 				uart0_puts("Sweep right\n");
// 				velocity(175,200);
// 				_delay_ms(2);
				j++;
				// 				itoa(j,R_v,10);
				// 				uart0_puts(R_v);
				// 				uart0_puts("\n");
			}
			if(R < 50 && C < 50 && L < 50){
				;
			}
			else{
				break;
			}
			i = 0;
			while((R < 50 && C < 50 && L < 50) && ( 10 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				velocity(175,200);
				angle_rotate(5);
// 				velocity(175,200);
// 				_delay_ms(2);
				i++;
				// 				itoa(i,L_v,10);
				// 				uart0_puts(L_v);
				// 				uart0_puts("\n");
				
			}
			if(R < 50 && C < 50 && L < 50){
				;
			}
			else{
				break;
			}
			
		}
	}
}
void sweep_invtd(void){                              //line follower fns starts
	forward();
	velocity(80,80);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	
	if(R > 50 && C > 50 && L > 50){
		uart0_puts("calibrating sweep\n");
		forward();
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		
		while(R > 50 && C > 50 && L > 50){

			uart0_puts("Sweep init\n");
			int i = 0;
			while((R > 50 && C > 50 && L > 50) && ( 10 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				angle_rotate(5);
				velocity(175,200);
				// 				velocity(175,200);
				//      			_delay_ms(2);
				i++;
				// 				itoa(i,L_v,10);
				// 				uart0_puts(L_v);
				// 				uart0_puts("\n");
				
			}
			if(R > 50 && C > 50 && L > 50){
				;
			}
			else{
				break;
			}
			int j  = 0;
			while((R > 50 && C > 50 && L > 50) && ( 10 > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				velocity(175,200);
				uart0_puts("Sweep right\n");
				angle_rotate(5);
				//
				// 				_delay_ms(2);
				j++;
				// 				itoa(j,R_v,10);
				// 				uart0_puts(R_v);
				// 				uart0_puts("\n");
			}
			if(R > 50 && C > 50 && L > 50){
				;
			}
			else{
				break;
			}
			j  = 0;
			while((R > 50 && C > 50 && L > 50) && ( 10 > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				velocity(175,200);
				angle_rotate(5);
				// 				uart0_puts("Sweep right\n");
				// 				velocity(175,200);
				// 				_delay_ms(2);
				j++;
				// 				itoa(j,R_v,10);
				// 				uart0_puts(R_v);
				// 				uart0_puts("\n");
			}
			if(R > 50 && C > 50 && L > 50){
				;
			}
			else{
				break;
			}
			i = 0;
			while((R > 50 && C > 50 && L > 50) && ( 10 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				velocity(175,200);
				angle_rotate(5);
				// 				velocity(175,200);
				// 				_delay_ms(2);
				i++;
				// 				itoa(i,L_v,10);
				// 				uart0_puts(L_v);
				// 				uart0_puts("\n");
				
			}
			if(R > 50 && C > 50 && L > 50){
				;
			}
			else{
				break;
			}
			
		}
	}
}
void follow_path(){

	int key = 0;
	uart0_puts("enter folw-path\n");
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	//R_v = R - 50;
	//L_v = L - 50;
	if(R < 100 && C > 100 && L < 100){
		//uart0_puts("forward\n");
		forward();
		velocity(200,200);
	}
	else if(R < 100 && C > 20 && L < 100){
		forward();
		velocity(200,200);
	}
	else if(R > 100 && C < 100 && L < 100){
		//uart0_puts("soft_right-0\n");
		soft_right();
		velocity(175,200);
	}
	else if(R > 20 && C < 100 && L < 100){
		//uart0_puts("soft_right-w\n");
		soft_right();
		velocity(175,200);
	}
// 	else if(R > 100 && C > 100 && L < 100){
// 		uart0_puts("soft_right-2\n");
// 		soft_right();
// 		//R_v += 30;
// 		velocity(100,100);
// 	}
	else if(R < 100 && C < 100 && L > 100){
		//uart0_puts("soft_left-0\n");
		soft_left();
		velocity(175,200);
	}
	else if(R < 100 && C < 100 && L > 20){
		//uart0_puts("soft_left-w\n");
		soft_left();
		velocity(175,200);
	}
// 	else if(R < 100 && C > 100 && L > 100){
// 		//uart0_puts("soft_right-2\n");
// 		soft_left();
// 		//L_v += 30;
// 		velocity(100,100);
// 	}
	/*else if(R > 100 && C > 100 && L > 100){
		//uart0_puts("Finding node\n");
		forward();
		_delay_ms(100);
		
// 		R = ADC_Conversion(1);
// 		C = ADC_Conversion(2);
// 		L = ADC_Conversion(3);
// 		itoa(L,strL,10);
// 		uart0_puts(strL);
// 		uart0_puts("\t");
// 		itoa(C,strC,10);
// 		uart0_puts(strC);
// 		uart0_puts("\t");
// 		itoa(R,strR,10);
// 		uart0_puts(strR);
// 		uart0_puts("\n");
		if(R > 130 && C > 130 && L > 130){
		   uart0_puts("node detected\n");
			stop();
			_delay_ms(1000);
		}
		
	}*/
	else if((R > 100 && C > 100)||(L > 100 && C > 100)){
		
		forward();
		_delay_ms(20);
		
		if((R > 100 && C > 100)||(L > 100 && C > 100)){
			uart0_puts("node detected\n");
			count=count+1;
			switch (count)
			{
				case 1: // code to be executed if n = 1;
				
				right_turn_for();
				
				

				
				//_delay_ms(225000);
				break;
				case 2: // code to be executed if n = 2;
				right_turn_for();
				break;
				case 3:
				
				left_turn_for();
				stop();
				_delay_ms(100);
				
				

				_delay_ms(100);
				w1pick();
				_delay_ms(250);
/*
// 			
*/
				right_turn();
				
		         break;
				 case 5:
				left_turn_for();
				stop();
				_delay_ms(100);
				
				

				_delay_ms(100);
				w2pick();
				_delay_ms(250);
				/*
				//
				*/
				right_turn();
				 break;
				 
				
				case 6:
				right_turn_for();
				break;
				case 7:
		stop();
		_delay_ms(1000);
		


		h4place();
		_delay_ms(250);
		right_turn();
				break;
				case 9:
				stop();
				
				_delay_ms(250);
				left_turn_for();
			


				h2place();
				_delay_ms(250);
/*
// 				sweep();
// 				_delay_ms(100);
*/
				right_turn();
				_delay_ms(250);
				case 11:
				right_turn_for();
				break;
				case 12:
				stop();
				_delay_ms(6000);
				break;
				
				
				default:forward_mm(100) ;  // code to be executed if n doesn't match any cases
				break;
			}
           
			

		}
		
	}
		
	else{
		uart0_puts("else\n");
		int k = 0;
		while( (R < 30 && C < 20 && L < 30) && k < max_tym){
				_delay_ms(1);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				uart0_putc(k);
				uart0_puts("\t k - value\n");
			    k++;
			}
		if (k >= max_tym){
			uart0_puts("sweep enter\n");
			sweep();
		}
	}
	/*else if(R < 130 && C < 130 && L < 130){
		uart0_puts("calibrating sweep\n");
		forward();
		_delay_ms(5);
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		
		while(R < 130 && C < 130 && L < 130){
			int i = 0;
			uart0_puts("Sweep initialised\n");
			
			for(i = 0;i < max_tym  ;i++){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				soft_left();
				velocity(100,100);
				_delay_ms(10);
				if(R < 130 && C < 130 && L < 130){
					continue;
				}
				else if(C>80){
					key = 1;
					break;
				}
			}
			int j  = 0;
		    for(j = 0;j < max_tym ;j++){
				right_back();
				velocity(80,80);
				_delay_ms(max_tym);
			    R = ADC_Conversion(1);
			    C = ADC_Conversion(2);
			    L = ADC_Conversion(3);
			    soft_right();
			    velocity(100,100);
			    _delay_ms(10);
				
			    if(R < 130 && C < 130 && L < 130){
				    continue;
			    }
			    else if(C>80){
				    key = 1;
				    break;
			    }
		    }
	
		}
	}*/
}
void inverted_path(){

	int key = 0;
	uart0_puts("enter invtd-path\n");
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	//R_v = R - 50;
	//L_v = L - 50;
	if(R > 100 && C < 100 && L > 100){
		//uart0_puts("forward\n");
		forward();
		velocity(200,200);
	}
	else if(R > 100 && C < 20 && L > 100){
		forward();
		velocity(200,200);
	}
	else if(R < 100 && C > 100 && L > 100){
		//uart0_puts("soft_right-0\n");
		soft_right();
		velocity(175,200);
	}
	else if(R < 20 && C > 100 && L > 100){
		//uart0_puts("soft_right-w\n");
		soft_right();
		velocity(175,200);
	}
// 	else if(R < 100 && C < 100 && L > 100){
// 		uart0_puts("soft_right-2\n");
// 		soft_right();
// 		//R_v += 30;
// 		velocity(100,100);
// 	}
	else if(R > 100 && C > 100 && L < 100){
		//uart0_puts("soft_left-0\n");
		soft_left();
		velocity(175,200);
	}
	else if(R > 100 && C > 100 && L < 20){
		//uart0_puts("soft_left-w\n");
		soft_left();
		velocity(175,200);
	}
	else if((R < 100 && C < 100)||(L < 100 && C < 100)){
		
		forward();
		_delay_ms(20);
		
		if((R < 100 && C < 100)||(L < 100 && C < 100)){
			uart0_puts("node detected\n");
			count=count+1;	
			forward_mm(100) ;  // code to be executed if n doesn't match any cases
			
           
			

		}
		
	}
	/*	
	else{
		uart0_puts("else\n");
		int k = 0;
		while( (R > 30 && C > 20 && L > 30) && k < max_tym){
				_delay_ms(1);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				uart0_putc(k);
				uart0_puts("\t k - value\n");
			    k++;
			}
		if (k >= max_tym){
			uart0_puts("sweep enter\n");
			sweep_invtd();
		}
	}*/
	
}
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	timer3_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}
void servo1test()
{
	servo_1(180);
	_delay_ms(400);
	servo_1(125);
	_delay_ms(1000);
	servo_1(180);
	_delay_ms(400);
	servo_1(125);
	_delay_ms(1000);
	servo_1(180);
	_delay_ms(400);
	servo_1(125);
	_delay_ms(250);
	servo_1(180);
	
	_delay_ms(250);
	
}
void servo4_160()
{
	int i;
	for(i=90;i<=170;i++)
	{
		servo_4(i);
		_delay_ms(30);
		
	}
	
}
void servo4_90()
{
	int i;
	for(i=170;i>=90;i--)
	{
		servo_4(i);
		_delay_ms(30);
		
	}
}
void ledon()
{
	
	DDRK=DDRK | 0x01;
	PORTK=PORTK & 0x01 ;
}
void ledoff()
{
	
	DDRK=DDRK | 0x01;
	PORTK=PORTK & 0x00 ;
}
void w1pick()
{
	 straight_position();
	 _delay_ms(500);
	 velocity(150,150);
	 back_mm(25);
	 _delay_ms(250);
	 downposn();
	 servo_2(140);
	 _delay_ms(1500);


	 servo_1(120);
	 servo3straight();
	 servo2straight();
	
	 servo2backgo();
	 _delay_ms(100);
	 servo_1(180);
	 _delay_ms(250);
	 servo2backstraight();
	 _delay_ms(250);
	 velocity(150,150);
	 forward_mm(15);
	 
	 
	
}
void w2pick()
{
	 straight_position();
	 _delay_ms(500);
	 velocity(150,150);
	 back_mm(25);
	 _delay_ms(250);
	 downposn();
	 servo_2(140);
	 _delay_ms(1500);
      servo_1(120);
	 servo3straight();
	 servo2straight();
	 servo4_160();         //
	 _delay_ms(250);
	 servo2backgo();
	 _delay_ms(100);
	 servo_1(180);
	 _delay_ms(250);
	 servo2backstraight();
	 _delay_ms(250);
	 straight_position();
	 velocity(150,150);
	  forward_mm(15);
	
	

}
void placesmallh()
{
	servo2backgo();
	_delay_ms(250);
	servo_1(120);
	_delay_ms(100);
	servo2backstraight();
	int i;
	for(i=150;i>=60;i--)
	{
		servo_3(i);             // first servo
		_delay_ms(20);
	}
	servo_1(180);
	_delay_ms(250);
	for(i=60;i<=150;i++)
	{
		servo_3(i);
		_delay_ms(15);
	}
	
	straight_position();
}
void h2place(){
	back_mm(40);
	_delay_ms(200);
	straight_position();  
	servo4_160();
	_delay_ms(200);
	servo2backgo();
	_delay_ms(200);
	servo2backstraight();
	servo4_90();
	int i;
	for(i=150;i>=60;i--)
	{
		servo_3(i);             // first servo
		_delay_ms(20);
	}
	_delay_ms(200);
	servo_1(180);
	_delay_ms(250);
	for(i=60;i<=150;i++)
	{
		servo_3(i);
		_delay_ms(15);
	}
	
	straight_position();
	
	forward_mm(32);
	straight_position();
	
}
void h4place()
{
	
	 	velocity(150,150);
	 	forward_mm(70); 
		 straight_position();
		 servo_1(180);             //for big house
	 	
	servo2backgo();
	_delay_ms(250);
	servo_1(120);
	_delay_ms(250);
	servo2backstraight();
	_delay_ms(100);
	for(int i=100;i>=0;i--)
	{
		servo_2(i);
		_delay_ms(20);
	}
	_delay_ms(100);
	servo_1(180);
	_delay_ms(250);
	straight_position();
	_delay_ms(200);
	
	back_mm(35);
	_delay_ms(250);
	stop();
}
int main()
{
	init_devices();
	uart0_init(UART_BAUD_SELECT(115200,(F_CPU)));
	uart0_flush();
	ledon();
	_delay_ms(500);
	ledoff();
	straight_position();
	velocity(150,150);
/*
// 	forward_mm(70);
// 	_delay_ms(200);
*/
	//left_turn();
 	while(1){
 		uart0_flush();
 		L = ADC_Conversion(3);	
 		C = ADC_Conversion(2);	
 		R = ADC_Conversion(1);
		flag = 0;
		itoa(L,strL,10);
 		uart0_puts(strL);
 		uart0_puts("\t");
 		itoa(C,strC,10);
 		uart0_puts(strC);
 		uart0_puts("\t");
 		itoa(R,strR,10);
 		uart0_puts(strR);
		uart0_puts("\n");		
		follow_path();
 	}

/*
//   _delay_ms(2000);
//   servo_1_free();
//   servo_2_free();
//   servo_3_free();
//   servo_4_free();
*/
 
}