
 /*
 * Team Id: eYRC #574
 * Author List: Abhishek gowda,Rohit Devar,Sanjay H B,
 * 
 * Filename: line_follower
 * Theme: Construct-O-Bot
 * Functions: grapher,junction_function,path_planner,sweep,sweep_lil,follow_path,
               ARM FUNCTIONS:pick,placestr,center to ground ,left_to_ground,leftmost_to_ground,right_to_ground,rightmost_to_ground,
                and other 6 functions for cm placing  from each left,right,centre,leftmost,rightmost to houses,str(arm initial posn) are important functions
* Global Variables: L,C,R(readings of line follower),C_max,C_min,L_max,L_min,R_max,R_min,(analog readings to know weather the robot is in white or black line)
 *                    
 */
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
void sweep_2C();
void sweep();
void sweep_lil();
void follow_path();
void buzzer_pin_config ();
void pickplacetrial();
void enter_invtd();

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
int cm[6][2],ccm=0,platform[5]={-1,-1,-1,-1,-1};
int house[5][2] = {
	{5,6},
	{3,2},
	{6,4},
	{1,2},
	{1,4}
};
int whb_path[15],travel_path[15],main_path[50];   //Initialise them to zero
int len_main_path = 0;
int travel_path_pref_l[]= {16,15,14,13,12,11,10,1,2,3,4,5,6}; //13
int travel_path_pref_r[]= {8,7,6,5,4,3,2,1,10,11,12,13,14}; //13
int len_tp = 13;
int path_pos = 0; // to keep the pos on  main_path
int cnt = 0; //to keep the count of no of cms on chasis(biased)
int white_del = 0; // status of white house(0 for not fulfilled 1 otherwise)


int whb1 = 3;   //special house first cm is 3,5,7,11
int whb2 = 13;
int wh_same_side = 0;  //set 1 if they are not on same side 0 if not  both on dft side
int whb_cm_count = 2; //no of cms for whb
int left_row_needs = 4;
int right_row_needs = 4;
int left_cms_to_pick = 4;  //no of cms to be picked up from left row
int right_cms_to_pick = 6; //no of cms to be picked up from right row
int house_height[]={0,1,1,0,0};
int whb_path1[] = {0,1,2,3,2,1,10,11,12,13,14,15,16,9};           //14
int whb_path2[] = {0,1,10,11,10,1,2,3,4,5,6,7,8,9};               //14
int whb_path3[] = {0,1,10,11,12,13,14,15,14,6,7,8,9};             //13
int whb_path4[] = {0,1,2,3,4,5,6,7,6,14,15,16,9};                 //13
int whb_path5[] = {0,1,10,11,12,13,14,15,14,6,5,6,7,8,9};           //15
int whb_path6[] = {0,1,10,11,12,13,14,15,14,6,5,6,7,8,9};         //15
int whb_path7[] = {0,1,2,3,4,5,6,7,6,14,13,14,15,16,9}; //15
int whb_path8[] = {0,1,2,3,4,5,6,7,8,9} ;    //10
int whb_path9[] = {0,1,10,11,12,13,14,15,16,9};  //10
int len_wp12 = 14,len_wp34 = 13,len_wp567 = 15,len_wp89 =10;
int left_pref = 0;
int right_pref = 0;
int colour,count=0,place1=-1,place2=-1,f=0;
int prv_pos,cur_pos,nxt_pos;

int error_flagR = 0;
int path_key = 0; // 0 for black line and 1 for white line
//right motor is faster than left one
int lmax_speed = 255;
int rmax_speed = .94557*255;  // rspeed = 0.938775 * lspeed
int C_max = 170;   // max of centre sensor value on full black
int C_mid = 120;
int C_min = 15;
int R_max = 200;  //full black
int L_max = 200;
int R_min = 15;     // black traces
int L_min = 15;
int R_mid = 150;
int L_mid = 150;
int swp_val = 50;// for all 3 R,L,C
int ijunction = 0;  // for keeping the count of junc in kadu
int max_tym = 35;


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
	DDRE  = DDRE | 0x08;  //making PORTe 3 pin output
	PORTE = PORTE | 0x08; //setting PORTe 3 pin to logic 1
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

	Direction &= 0x33; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xCC; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)
{
	motion_set (0x12);
}

void soft_right (void)
{
	motion_set (0x10);
}

void soft_left (void)
{
	motion_set (0x02);
}

void right (void)
{
	motion_set (0x11);
}

void left (void)
{
	motion_set (0x22);
}

void backward (void)
{
	motion_set (0x21);
}

void soft_right_2(void)
{
	motion_set(0x20);
	
}

void soft_left_2(void)
{
	motion_set(0x01);
	
}
void stop (void)
{
	motion_set (0x33);
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
		if((ShaftCountRight >= ReqdShaftCountInt)&&(ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
		break;
		}
	}

	 //Stop robot
}

//functions used to rotate robot to certain angle using encoder

void left_turn_encoder()
{
	unsigned long int ReqdLeftCount = 360;//355
	unsigned long int ReqdRightCount = 360;//355
	

	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	left();
	velocity(rmax_speed,lmax_speed);
	while (1)
	{
		if((ShaftCountRight >=  ReqdRightCount)&&(ShaftCountLeft >=  ReqdLeftCount))
		{
			stop();
			break;
		}
	}
	//Stop robot
}                              // functions                      //Function used for moving robot forward by specified distance       
void right_turn_encoder()
{
	unsigned long int ReqdLeftCount = 340;//335
	unsigned long int ReqdRightCount = 340;//335
	

	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	right();
	velocity(rmax_speed,lmax_speed);
	while (1)
	{
		if((ShaftCountRight >=  ReqdRightCount)&&(ShaftCountLeft >=  ReqdLeftCount))
		{
			stop();
			break;
		}
	}
	//Stop robot
}

//Function used for moving robot forward by specified distance 

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM /0.35; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountLeft = 0;
	ShaftCountRight = 0;
	while(1)
	{
		if((ShaftCountLeft > ReqdShaftCountInt))//||(ShaftCountLeft >= ReqdShaftCountInt))
		{
			stop();
			break;
		}
	}
	 //Stop robot
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
// void right_turn_for(){
// 	 stop();
// 	 _delay_ms(250);
// 	// sweep_2C();
// 	forward_mm(70);
// 	//sweep_2C();
// 	left();
// 	angle_rotate(165);
// 	}

//functions used to turn robot b using delay and by detecting black line
void right_turn_for(){
	 stop();
	_delay_ms(2);
	velocity(rmax_speed*.7,lmax_speed*.7);
	right();
	/*angle_rotate(80);*/
	_delay_ms(300);
	velocity(rmax_speed*.65,lmax_speed*.65);
	_delay_ms(20);
	velocity(rmax_speed*.5,lmax_speed*.5);
	_delay_ms(80);
	//velocity(rmax_speed*.35,lmax_speed*.35);
	//_delay_ms(80);
	 		// 	Use below if enc250oder gives wrong by wls
			  L = ADC_Conversion(3);
			 R = ADC_Conversion(1);
			 C = ADC_Conversion(2);
	 		while((C < 30) && (R < 30) && (L < 30)){
		 		velocity(rmax_speed*.45,lmax_speed*.45);
		 		right();
				_delay_ms(1);
				L = ADC_Conversion(3);
		 		C = ADC_Conversion(2);
		 		R = ADC_Conversion(1);
	 		}
		stop();
		_delay_ms(30);
		velocity(rmax_speed,lmax_speed);
}
void left_turn_for(){
		 stop();
		 _delay_ms(2);
		 velocity(rmax_speed*.7,lmax_speed);
		 left();
		 /*angle_rotate(80);*/
		 _delay_ms(300);
		 velocity(rmax_speed*.65,lmax_speed*.65);
		 _delay_ms(20);
		 velocity(rmax_speed*.5,lmax_speed*.5);
		 _delay_ms(80);
		 velocity(rmax_speed*.45,lmax_speed*.45);
		//_delay_ms(80);
		 // 	Use below if enc250oder gives wrong by wls
		 R = ADC_Conversion(1);
		 C = ADC_Conversion(2);
		 L = ADC_Conversion(3);
		 while((C < 30) && (R < 30) && (L < 30)){
			 velocity(rmax_speed*.45,lmax_speed*.45);
			 left();
			 _delay_ms(1);
			 R = ADC_Conversion(1);
			 C = ADC_Conversion(2);
			 L = ADC_Conversion(3);

		 }
		 stop();
		 _delay_ms(30);
		 velocity(rmax_speed,lmax_speed);
}
void right_turn(){
 	stop();
	_delay_ms(100);
	right_turn_encoder();
	_delay_ms(50);
// 	velocity(rmax_speed,lmax_speed);
 	//right();

		//stop();
}

void left_turn(){
	stop();
	_delay_ms(100);
	left_turn_encoder();
	_delay_ms(50);

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
	buzzer_pin_config();
	sei();   //Enables the global interrupts
}
// the bleow three are buzzer funnctions which makes buzzer on and off
void buzzer_pin_config ()
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
void U_turn(){
	left_turn_for();
	left_turn_for();
}
void declaration(){
	//cm declaration
	for(int i = 0;i<6;i++){
		for(int j = 0;j<2;j++){
			cm[i][j] = 1;
		}
	}
	
}
void travel_path_setter1(){
	int temp = 0;
	//by default left_pref is on
	if(left_row_needs > right_row_needs){
		left_pref = 1;
		//travel_path_pref_l= {16,15,14,13,12,11,10,1,2,3,4,5,6};
	}
	else if(left_row_needs < right_row_needs){
		right_pref = 1;
		//	travel_path_pref_r= {8,7,6,5,4,3,2,1,10,11,12,13,14};
	}
	else if(left_row_needs == right_row_needs){
		if(left_cms_to_pick > right_cms_to_pick)
		left_pref = 1;
		else if(right_cms_to_pick > left_cms_to_pick)
		right_pref = 1;
	}
	// or else both stays 0 by default make pref_r = 1
	if(left_pref == 1){
		if(whb1 < whb2){
			temp = whb1;
			whb1 = whb2;
			whb2 = temp;
		}
	}
	printf("lp-%d rp-%d\n",left_pref,right_pref);
	if(right_pref == 1){
		if(whb1 > whb2){
			printf("\n");
			printf("t %d 1 %d 2 %d\n",temp,whb1,whb2);
			temp = whb1;
			whb1 = whb2;
			whb2 = temp;
			printf("t %d 1 %d 2 %d\n",temp,whb1,whb2);
		}
	}
	
}
void whb_setter2(){
	if((whb1 == 3 && ((whb2 == 15) || (whb2 == 13) || (whb2 == 11))) )
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp12;i++){
			whb_path[i] = whb_path1[i];
		}
		
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 1;
		left_pref = 0;
		
	}
	else if((whb1 == 11 && ((whb2 == 7) || (whb2 == 3))) || (whb1 == 5 && whb2 == 11))
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp12;i++){
			whb_path[i] = whb_path2[i];
		}
		
		
		
		
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 0;
		left_pref = 1;
		
	}
	else if((whb1 == 7 && whb2 == 13) || (whb1 == 7 && whb2 == 11) || (whb1 == 7 && whb2 == 15))
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp34;i++){
			whb_path[i] = whb_path3[i];
		}
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 1;
		left_pref = 0;
	}
	else if((whb1 == 15 && whb2 == 5) || (whb1 == 15 && whb2 == 3) ||(whb1 == 15 && whb2 == 7))
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp34;i++){
			whb_path[i] = whb_path4[i];
		}
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 0;
		left_pref = 1;
		
	}
	else if((whb1 == 5 && whb2 == 13) || (whb1 == 5 && whb2 == 11))
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp567;i++){
			whb_path[i] = whb_path5[i];
		}
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 1;
		left_pref = 0;
	}
	else if(whb1 == 5 && whb2 == 15)
	{
		//copy the respective path to array whb_path(max size 15)
		int i = 0;
		for(i = 0;i<len_wp567;i++){
			whb_path[i] = whb_path6[i];
		}
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 1;
		left_pref = 0;
		
	}
	else if((whb1 == 13 && whb2 == 5) || (whb1 == 13 && whb2 == 7) || (whb1 == 13 && whb2 == 3))
	{
		int i = 0;
		for(i = 0;i<len_wp567;i++){
			whb_path[i] = whb_path7[i];
		}
		//rest set to -1
		while(i<15){
			whb_path[i] = -1;
			i++;
		}
		right_pref = 0;
		left_pref = 1;
	}
	else{
		printf("wrong combination error\n");
		exit(0);
	}
	
}
void whb_setter3(){
	if(whb_cm_count == 1){
		if(whb1 == 3 || whb1 == 5 || whb1 == 7){
			int i = 0;
			for(i = 0;i<len_wp89;i++){
				whb_path[i] = whb_path8[i];
			}
			//rest set to -1
			while(i<15){
				whb_path[i] = -1;
				i++;
			}
			right_pref = 0;
			left_pref = 1;
		}
		else if((whb1 == 11 || whb1 == 13 || whb1 == 15)){
			int i = 0;
			for(i = 0;i<len_wp89;i++){
				whb_path[i] = whb_path9[i];
			}
			//rest set to -1
			while(i<15){
				whb_path[i] = -1;
				i++;
			}
			right_pref = 1;
			left_pref = 0;
		}
	}
	else if(whb_cm_count == 2){
		if((whb1 == 3 || whb1 == 5 || whb1 == 7)&&(whb2 == 3 || whb2 == 5 || whb2 == 7)&&(whb1 != whb2)){
			int i = 0;
			for(i = 0;i<len_wp89;i++){
				whb_path[i] = whb_path8[i];
			}
			//rest set to -1
			while(i<15){
				whb_path[i] = -1;
				i++;
			}
			right_pref = 0;
			left_pref = 1;
		}
		else if((whb1 == 11 || whb1 == 15 || whb1 == 13)&&(whb2 == 13 || whb2 == 15 || whb2 == 11)&&(whb1 != whb2)){
			int i = 0;
			for(i = 0;i<len_wp89;i++){
				whb_path[i] = whb_path9[i];
			}
			//rest set to -1
			while(i<15){
				whb_path[i] = -1;
				i++;
			}
			right_pref = 1;
			left_pref = 0;
		}
		
	}
}
void path_merger(){
	int i,j,k;
	for(i = 0;i<15;i++){
		if(whb_path[i] >= 0){
			main_path[i] = whb_path[i];
		}
		else{
			k = i;
			break;
		}
	}
	for(i = 0;i<15;i++,k++){
		if(travel_path[i] >= 0){
			main_path[k] = travel_path[i];
		}
		else{
			j = k;
			break;
		}
	}
	//for left out cm shortest path
	// for(i = 0;i<15;i++,j++){
	//     if(shortest_path[i] >= 0){
	//         main_path[j] = shortest_path[i];
	//     }
	//     else{
	//         break;
	//     }
	// }
	len_main_path = j;
}
void path_planner(){
	printf("w1-%d w2-%d\n",whb1,whb2);
	if(wh_same_side != 1 && whb_cm_count == 2){
		whb_setter3();
	}
	printf("w1-%d w2-%d\n",whb1,whb2);
	if(whb_cm_count == 1 || wh_same_side == 1){
		whb_setter3();
	}
	else{
		whb_setter2();
		printf("w1-%d w2-%d\n",whb1,whb2);
	}
	int i = 0;
	if (right_pref == 1){
		for(i= 0;i<len_tp;i++){
			travel_path[i] = travel_path_pref_r[i];
		}
	}
	else if(left_pref == 1){
		for(i= 0;i<len_tp;i++){
			travel_path[i] = travel_path_pref_l[i];
		}
	}
	//rest set to -1
	while(i<15){
		travel_path[i] = -1;
		i++;
	}
	path_merger();
	printf("w1-%d w2-%d\n",whb1,whb2);
}


void grapher()
{
	printf("ppos %d cpos %d npos %d \n",prv_pos,cur_pos,nxt_pos);
	if(prv_pos == nxt_pos){
		stop();
	}
	else if(cur_pos == 1 && prv_pos == 0){
		if(nxt_pos == 2){
			right_turn_for();
		}
		else if(nxt_pos == 10 ){
			left_turn_for();
		}
	}
	else if(cur_pos == 2){
		if(nxt_pos == 3){
			right_turn_for();
		}
		else if(nxt_pos == 1){
			left_turn_for();
		}
	}
	else if(cur_pos == 3){
		if(prv_pos == 2){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 4){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 2){
				left_turn_for();
			}
			else if(nxt_pos == 4){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 4){
				left_turn_for();
			}
			else if(nxt_pos == 2){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 4){
		if(prv_pos == 3){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
		}
		else if(prv_pos == 5){
			if(nxt_pos == 'Q'){
				right_turn_for();
			}

		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 3){
				left_turn_for();
			}
			else if(nxt_pos == 5){
				right_turn_for();
			}
		}
	}
	else if(cur_pos == 5){
		if(prv_pos == 4){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 6){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 4){
				left_turn_for();
			}
			else if(nxt_pos == 6){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 6){
				left_turn_for();
			}
			else if(nxt_pos == 4){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 6){
		if(prv_pos == 5){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 14 ){
				right_turn_for();
			}
		}
		else if(prv_pos == 7){
			
			if(nxt_pos == 14 ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 5){
				left_turn_for();
			}
			else if(nxt_pos == 7){
				right_turn_for();
			}
			else if(nxt_pos == 14 ){
				U_turn();
			}
		}
		else if(prv_pos == 14 ){
			if(nxt_pos == 5){
				left_turn_for();
			}
			else if(nxt_pos == 7){
				right_turn_for();
			}
		}
	}
	else if(cur_pos == 7){
		if(prv_pos == 6){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 8){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 6){
				left_turn_for();
			}
			else if(nxt_pos == 8){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 8){
				left_turn_for();
			}
			else if(nxt_pos == 6){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 8){
		if(nxt_pos == 9){
			right_turn();
			enter_invtd();
		}
		else if(nxt_pos == 7){
			left_turn_for();
		}
	}
	else if(cur_pos == 9){
		if(prv_pos == 8){
			if(nxt_pos == 'I' )
			right_turn();
		}
		if(prv_pos == 'I' ){
			if(nxt_pos == 8){
				right_turn();
				right();
				angle_rotate(10);
			}
			else if(nxt_pos == 16 ){
				left_turn();
				left();
				angle_rotate(10);
			}
		}
	}
	else if(cur_pos == 16 ){
		if(nxt_pos == 9){
			left_turn();
			enter_invtd();
		}
		else if(nxt_pos == 15 ){
			left_turn_for();
		}
	}
	else if(cur_pos == 15 ){
		if(prv_pos == 16 ){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 14 ){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 16 ){
				left_turn_for();
			}
			else if(nxt_pos == 14 ){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 14 ){
				left_turn_for();
			}
			else if(nxt_pos == 16 ){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 14 ){
		if(prv_pos == 15 ){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 6){
				right_turn_for();
			}
		}
		else if(prv_pos == 13 ){
			if(nxt_pos == 6){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 15 ){
				left_turn_for();
			}
			else if(nxt_pos == 13 ){
				right_turn_for();
			}
			else if(nxt_pos == 6){
				U_turn();
			}
		}
		else if(prv_pos == 6){
			if(nxt_pos == 15 ){
				left_turn_for();
			}
			else if(nxt_pos == 13 ){
				right_turn_for();
			}
		}
	}
	else if(cur_pos == 13 ){
		if(prv_pos == 14 ){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 12 ){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 14 ){
				left_turn_for();
			}
			else if(nxt_pos == 12 ){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 12 ){
				left_turn_for();
			}
			else if(nxt_pos == 14 ){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 12 ){
		if(prv_pos == 13 ){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
		}
		else if(prv_pos == 11 ){
			if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 13 ){
				left_turn_for();
			}
			else if(nxt_pos == 11 ){
				right_turn_for();
			}
		}
	}
	else if(cur_pos == 11 ){
		if(prv_pos == 12 ){
			if(nxt_pos == 'Q'){
				left_turn_for();
			}
			else if(nxt_pos == 'I' ){
				right_turn_for();
			}
		}
		else if(prv_pos == 10 ){
			if(nxt_pos == 'I' ){
				left_turn_for();
			}
			else if(nxt_pos == 'Q'){
				right_turn_for();
			}
		}
		else if(prv_pos == 'Q'){
			if(nxt_pos == 12 ){
				left_turn_for();
			}
			else if(nxt_pos == 10 ){
				right_turn_for();
			}
			else if(nxt_pos == 'I' ){
				U_turn();
			}
		}
		else if(prv_pos == 'I' ){
			if(nxt_pos == 10 ){
				left_turn_for();
			}
			else if(nxt_pos == 12 ){
				right_turn_for();
			}
			else if(nxt_pos == 'Q'){
				U_turn();
			}
		}
	}
	else if(cur_pos == 10 ){
		if(nxt_pos == 1){
			right_turn_for();
		}
		else if(nxt_pos == 11 ){
			left_turn_for();
		}
	}
	else{
		printf("go straight\n");
	}
}

//function which brings arm to steady position 
void str()
{
	
	_delay_ms(50);
	//servo_1(180);
	servo_2(80);
	servo_3(80);
	servo_4(95);
	//forward bending fro, 75>
	
	
}
// this function makes arm to come at pick position 
void pickposn()
{
	
	str();
	
	//_delay_ms(100);
	
	int i;
	for( i=80;i<=140;i++)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(10);
	}
    
	for(int i=140;i<=220;i++)
	{
		
		servo_2(i);
		_delay_ms(20);
	}
	

	
	
	
	
}
//function makes the  arm to place cm at white house
void white_house()
{
	back_mm(70);
	stop();                                                            //arm will stsrt froms str posn with 85degreee servo 4  ....place and come to str posn
	servo_1(90);
	_delay_ms(25);
	servo_4(85);                                                         //as arm is not straight so to keep straight
	_delay_ms(50);
	servo_2(80);
	servo_3(80);
	
	_delay_ms(30);
	for(int i=80;i<=90;i++)
	{
		servo_3(i);
		_delay_ms(1);
		
	}
	for(int i=90;i<=118;i++)           // first 115
	{
		servo_3(i);
		_delay_ms(24);
		
	}
	
	for(int i=80;i<=160;i++)
	{
		servo_2(i);
		_delay_ms(1);
		
	}
	for(int i=160;i<=230;i++)
	{
		servo_2(i);
		_delay_ms(50);
		
	}
	servo_2_free();
	_delay_ms(1000);
	//for placing correctly
	_delay_ms(100);
	

	for (int i=90;i<=120;i++)
	{
		servo_1(i);
		_delay_ms(100);
		
	}
	for(int i=230;i>=200;i--)
	{
		
		servo_2(i);
		_delay_ms(70);
		
	}
	for(int i=200;i>=180;i--)
	{
		
		servo_2(i);
		_delay_ms(25);
		
	}
	
	for(int i=180;i>=80;i--)
	{

		
		servo_2(i);
		_delay_ms(1);
		
	}
	
	
	for(int i=118;i>=100;i--)
	{
		servo_3(i);
		_delay_ms(20);
	}
	for(int i=90;i>=80;i--)
	{
		servo_3(i);
		_delay_ms(10);
	}
	
	_delay_ms(200);
	
	forward_mm(60);
	
}
//function makes the  arm to place cm at small house
void small_H()
{
	back_mm(15); 
	stop();                                                            //arm will stsrt froms str posn with 85degreee servo 4  ....place and come to str posn
	servo_1(90);
	_delay_ms(25);
	servo_4(85);                                                         //as arm is not straight so to keep straight
	_delay_ms(50);
	servo_2(80);
	servo_3(80);
	
	_delay_ms(30);
	for(int i=80;i<=90;i++)
	{
		servo_3(i);
		_delay_ms(1);
		
	}
	for(int i=90;i<=120;i++)           // first 115
	{
		servo_3(i);
		_delay_ms(24);
		
	}
	
	for(int i=80;i<=160;i++)
	{
		servo_2(i);
		_delay_ms(1);
		
	}
	for(int i=160;i<=230;i++)
	{
		servo_2(i);
		_delay_ms(50);
		
	}
	servo_2_free();
	_delay_ms(1000);
	//for placing correctly
	_delay_ms(100);
	

	for (int i=90;i<=120;i++)
	{
		servo_1(i);
		_delay_ms(100);
		
	}
	for(int i=230;i>=200;i--)
	{
		
		servo_2(i);
		_delay_ms(70);
		
	}
	for(int i=200;i>=180;i--)
	{
		
		servo_2(i);
		_delay_ms(25);
		
	}
	
	for(int i=180;i>=80;i--)
	{

		
		servo_2(i);
		_delay_ms(1);
		
	}
	
	
	for(int i=120;i>=100;i--)
	{
		servo_3(i);
		_delay_ms(20);
	}
	for(int i=90;i>=80;i--)
	{
		servo_3(i);
		_delay_ms(10);
	}
	
	_delay_ms(200);
	
	forward_mm(30);
	
}
//function makes the  arm to place cm at small house
void small_H_speed()
{
	//back_mm(00);             // given 20                                                     //arm will stsrt froms str posn with 85degreee servo 4  ....place and come to str posn
	servo_1(85);
	_delay_ms(25);
	servo_4(90);                                                         //as arm is not straight so to keep straight
	_delay_ms(25);
	servo_2(80);
	servo_3(80);
	
	_delay_ms(30);
	for(int i=80;i<=80;i++)
	{
		servo_3(i);
		//servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=80;i<=160;i++)
	{
		servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=160;i<=260;i++)
	{
		servo_2(i);
		_delay_ms(15);
		servo_2_free();
	}
	//_delay_ms(100);
	
    servo_2_free();
	_delay_ms(1000);
	
	//_delay_ms(100);
	

	for (int i=85;i>=55;i--)
	{
		servo_1(i);
		servo_2_free();
		_delay_ms(150);
		
	}
	for(int i=230;i>=210;i--)
	{
		
		servo_2(i);
		_delay_ms(20);
		
	}
	for(int i=210;i>=105;i--)
	{
		
		servo_2(i);
		_delay_ms(1);
		
	}
	

	
	
	for(int i=105;i>=80;i--)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(10);
	}
}
//function makes the  arm to place cm at small house
void small_H_slow()
{
	back_mm(13);
	//back_mm(00);             // given 20                                                     //arm will stsrt froms str posn with 85degreee servo 4  ....place and come to str posn
	servo_1(85);
	_delay_ms(25);
	servo_4(90);                                                         //as arm is not straight so to keep straight
	_delay_ms(25);
	servo_2(80);
	servo_3(80);
	
	_delay_ms(30);
	for(int i=80;i<=100;i++)
	{
		servo_3(i);
		//servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=80;i<=160;i++)
	{
		servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=160;i<=260;i++)
	{
		servo_2(i);
		_delay_ms(15);
		servo_2_free();
	}
	//_delay_ms(100);
	
	servo_2_free();
	_delay_ms(1500);
	
	//_delay_ms(100);
	

	for (int i=85;i>=55;i--)
	{
		servo_1(i);
		servo_2_free();
		_delay_ms(150);
		servo_2_free();
		
	}
	for(int i=230;i>=210;i--)
	{
		
		servo_2(i);
		_delay_ms(20);
		
	}
	for(int i=210;i>=105;i--)
	{
		
		servo_2(i);
		_delay_ms(1);
		
	}
	

	
	
	for(int i=105;i>=80;i--)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(1);
	}
	
	forward_mm(10);
}
//function makes the  arm to place cm at small white  house
void small_H_slow_white()
{
	back_mm(13);
	//back_mm(00);             // given 20                                                     //arm will stsrt froms str posn with 85degreee servo 4  ....place and come to str posn
	servo_1(85);
	_delay_ms(25);
	servo_4(70);                                                         //as arm is not straight so to keep straight
	_delay_ms(25);
	servo_2(80);
	servo_3(80);
	
	_delay_ms(30);
	for(int i=80;i<=100;i++)
	{
		servo_3(i);
		//servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=80;i<=160;i++)
	{
		servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=160;i<=260;i++)
	{
		servo_2(i);
		_delay_ms(15);
		servo_2_free();
	}
	//_delay_ms(100);
	
	servo_2_free();
	_delay_ms(1500);
	
	//_delay_ms(100);
	

	for (int i=85;i>=55;i--)
	{
		servo_1(i);
		servo_2_free();
		_delay_ms(150);
		servo_2_free();
		
	}
	for(int i=230;i>=210;i--)
	{
		
		servo_2(i);
		_delay_ms(20);
		
	}
	for(int i=210;i>=105;i--)
	{
		
		servo_2(i);
		_delay_ms(1);
		
	}
	

	
	
	for(int i=105;i>=80;i--)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(1);
	}
	
	forward_mm(10);
}
// function used to pick cm from ground for first time of particular colour
void cm_pick_1()
{
	servo_1(30);
    _delay_ms(250);	
	str();
	pickposn();
	 _delay_ms(300);
    servo_1(85);
 _delay_ms(600);
for(int i=220;i>=140;i--)
	{
		servo_2(i);
		_delay_ms(10);
	}
	for(int i=140;i>=80;i--)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(20);
	}
}
// function used to pick cm from ground for second time after 180 degere rotation of bot
void cm_pick_2()
{
	servo_1(0);
	_delay_ms(250);
	str();
	pickposn();
	_delay_ms(300);
	servo_1(85);
	_delay_ms(600);
	for(int i=220;i>=140;i--)
	{
		servo_2(i);
		_delay_ms(10);
	}
	for(int i=140;i>=80;i--)
	{
		servo_3(i);
		servo_2(i);
		_delay_ms(20);
	}
}
// function used to place the picked cm back of platform
void back_place(){
	for(int i=80;i<=100;i++)
	{
		servo_3(i);
		_delay_ms(30);
	}
	
	
	
	
	
	for(int i=80;i>=30;i--)
	{
		servo_2(i);
		_delay_ms(10);
	}
	
	for(int i=20;i>=0;i--)
	{
		servo_2(i);
		_delay_ms(30);             //first 20
	}
	_delay_ms(250);
    for(int i=85;i>=55;i--)
	{
		servo_1(i);
		_delay_ms(20);
	}
/*
// 	for(int i=85;i<=0;i--){
// 		servo_1(i);
// 		_delay_ms(25);
// 	}
*/
	
	
	for(int i=0;i<=30;i++)
	{
		servo_2(i);
		_delay_ms(1);
	}
	for(int i=30;i<=80;i++)
	{
		servo_2(i);
		_delay_ms(1);


	}
	for(int i=100;i>=80;i--)
	{
		servo_3(i);
		_delay_ms(1);
	}
	_delay_ms(250);
}
//function used to pick the cms from platforms
void back_pick(){
	
	servo_1(55);
	_delay_ms(100);
	
	servo_3(85);
	for(int i=80;i>=30;i--)
	{
		servo_2(i);
		_delay_ms(10);
	}
	for(int i=30;i>=0;i--)
	{
		servo_2(i);
		_delay_ms(50);               //first 50
	}
	
	servo_1(85);
	_delay_ms(100);
	for(int i=0;i<=30;i++)
	{
		servo_2(i);
		_delay_ms(10);
	}
	for(int i=30;i<=80;i++)
	{
		servo_2(i);
		_delay_ms(10);
	}
	
	_delay_ms(500);
}
//function makes the  arm to place cm at big white house for first time
void big_H_white_speed()
{
	servo_1(85);
	_delay_ms(50);
	//servo_1(180);
	servo_4(95);
	
	servo_2(80);
	servo_3(80);
	servo_4(75);
	
	for(int i=80;i>=70;i--)
	{
		servo_3(i);
		_delay_ms(10);
		
	}
	
	for(int i=90;i<=100;i++)
	{
		servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=160;i<=210;i++)
	{
		servo_2(i);
		//servo_2_free();
		_delay_ms(25);
		
	}
	for(int i=210;i<=260;i++)
	{
		servo_2(i);
		servo_2_free();
		_delay_ms(50);
	}
	servo_2_free();
	_delay_ms(1500);
	
	for (int i=85;i>=55;i--)
	{
		
		servo_1(i);
		servo_2_free();
		_delay_ms(100);
		
	}
	for(int i=220;i>=180;i--)
	{
		servo_2(i);
		_delay_ms(30);
		
	}
	/*
	// 		for(int i=65;i>=55;i--)
	// 		{
	// 			servo_1(i);
	//
	// 			_delay_ms(1);
	// 		}
	*/
	for(int i=180;i>=80;i--)
	{
		servo_2(i);
		_delay_ms(1);
	}
	
}
//function makes the  arm to place cm at big white house for second  time
void big_H_white_slow()
{
	servo_1(85);
	_delay_ms(50);
	//servo_1(180);
	servo_4(95);
	
	servo_2(80);
	servo_3(80);
	servo_4(70);
	
	for(int i=80;i>=70;i--)
	{
		servo_3(i);
		_delay_ms(10);
		
	}
	
	for(int i=90;i<=100;i++)
	{
		servo_2(i);
		_delay_ms(5);
		
	}
	for(int i=160;i<=210;i++)
	{
		servo_2(i);
		//servo_2_free();
		_delay_ms(25);
		
	}
	for(int i=210;i<=260;i++)
	{
		servo_2(i);
		servo_2_free();
		_delay_ms(50);
	}
	servo_2_free();
	_delay_ms(1500);
	
	for (int i=85;i>=55;i--)
	{
		
		servo_1(i);
		servo_2_free();
		_delay_ms(100);
		
	}
	for(int i=220;i>=180;i--)
	{
		servo_2(i);
		_delay_ms(30);
		
	}
	/*
	// 		for(int i=65;i>=55;i--)
	// 		{
	// 			servo_1(i);
	//
	// 			_delay_ms(1);
	// 		}
	*/
	for(int i=180;i>=80;i--)
	{
		servo_2(i);
		_delay_ms(1);
	}
}
//function makes the  arm to place cm at big normal house for second  time
void big_H_slow()
	{
		servo_1(85);
		_delay_ms(50);
		//servo_1(180);
		servo_4(95);
		
		servo_2(80);
		servo_3(80);
		servo_4(95);
		
		for(int i=80;i>=70;i--)
		{
			servo_3(i);
			_delay_ms(10);
			
		}
		
		for(int i=90;i<=100;i++)
		{
			servo_2(i);
			_delay_ms(5);
			
		}
		for(int i=160;i<=210;i++)
		{
			servo_2(i);
			//servo_2_free();
			_delay_ms(25);
			
		}
		for(int i=210;i<=260;i++)
		{
			servo_2(i);
			servo_2_free();
			_delay_ms(50);
		}
		servo_2_free();
		_delay_ms(1500);
		
		for (int i=85;i>=55;i--)
		{
			
			servo_1(i);
			servo_2_free();
			_delay_ms(100);
			
		}
		for(int i=220;i>=180;i--)
		{
			servo_2(i);
			_delay_ms(30);
			
		}
/*
// 		for(int i=65;i>=55;i--)
// 		{
// 			servo_1(i);
// 			
// 			_delay_ms(1);
// 		}
*/
		for(int i=180;i>=80;i--)
		{
			servo_2(i);
			_delay_ms(1);
		}
	}
	//function makes the  arm to place cm at big normal  house for first  time
void big_H_speed()
{
	big_H_slow();
}

/*
 * Function Name:leftmost_from_ground
 * Input: it takes "a" as a input 
 * Output: none
 * Logic: it is used to pick cm from ground and place it in leftmost part of platform
 * 
 * Example Call: it is called in pick function 
*/
void leftmost_from_ground(int a)
{
	servo_1(55);
	_delay_ms(250);
	if(a==1)
	cm_pick_1();
	else
	cm_pick_2();
	_delay_ms(500);
	str();
	_delay_ms(250);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_place();
	_delay_ms(100);
	
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	str();
}
// these functions are use to place cms from leftmost place of platform to the respective houses
void leftmost_to_big_speed()
{
	servo_1(55);

	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	//big_H();
	big_H_speed();
}
void leftmost_to_big_slow(){
	servo_1(55);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	big_H_slow();
	
}
void leftmost_to_small_speed()
{
	servo_1(55);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_speed();
	
}
void leftmost_to_small_slow()
{
	servo_1(55);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_slow();
}
void leftmost_to_white_small_house_speed()
{
	servo_1(55);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
}
void leftmost_to_white_small_house_slow()
{
	//back_mm(45);
	stop();
	_delay_ms(100);
	servo_1(55);
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
	
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
	
}
void leftmost_to_white_big_house_speed()
{
	
	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
back_mm(35);
	big_H_speed();
	forward_mm(25);
}
void leftmost_to_white_big_house_slow()
{

	for(int i=95;i<=185;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=185;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(35);
	big_H_white_slow();
	forward_mm(25);
}

/*
 * Function Name:left_from_ground
 * Input: it takes "a" as a input 
 * Output: none
 * Logic: it is used to pick cm from ground and place it in left part of platform
 * 
 * Example Call: it is called in pick function 
*/
void left_from_ground(int a)
{
	servo_1(55);
	_delay_ms(100);
	if(a==1)
	cm_pick_1();
	else
	cm_pick_2();
	_delay_ms(500);
	str();
	_delay_ms(250);
	
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_place();
	
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
}
// these functions are use to place cms from centre place of platform to the respective houses
void left_to_big_speed()
{
	servo_1(55);

	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	//big_H();
	big_H_speed();
}
void left_to_big_slow(){
	servo_1(55);
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	big_H_slow();
	
}
void left_to_small_speed()
{
	servo_1(55);
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_speed();
	
}
void left_to_small_slow()
{
	servo_1(55);
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
	small_H_slow();
}
void left_to_white_small_house_speed()
{
	servo_1(55);
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
}
void left_to_white_small_house_slow()
{
	//back_mm(45);
	stop();
	_delay_ms(100);
	servo_1(55);
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
	_delay_ms(250);
	back_mm(10);
	small_H_slow_white();
	forward_mm(8);
	stop();
	
}
void left_to_white_big_house_speed()
{
	
	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
	back_mm(35);
	big_H_speed();
	forward_mm(25);
}
void left_to_white_big_house_slow()
{

	for(int i=95;i<=140;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=140;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(0);
	big_H_white_slow();
	forward_mm(0);
}

/*
 * Function Name:centre_from_ground
 * Input: it takes "a" as a input 
 * Output: none
 * Logic: it is used to pick cm from ground and place it in center part of platform
 * 
 * Example Call: it is called in pick function 
*/
void centre_from_ground(int a)
{
	servo_1(40);
	_delay_ms(250);
	if(a==1)
	cm_pick_1();
	else
	cm_pick_2();
	_delay_ms(250);
	servo_4(90);
	back_place();
	servo_4(95);
}
// these functions are use to place cms from centre place of platform to the respective houses
void centre_to_big_speed()
{
	servo_1(55);
	servo_4(87);
	back_pick();
	
	big_H_speed();
}
void centre_to_big_slow(){
	servo_1(55);
	servo_4(87);
	back_pick();
	big_H_slow();
}
void centre_to_small_speed()
{
	servo_1(55);
	servo_4(87);
	
	back_pick();
	small_H_speed();
	
}
void centre_to_small_slow()
{
	servo_1(55);
	servo_4(87);
	
	back_pick();
	small_H_slow();
}
void centre_to_white_small_house_speed()
{
	servo_1(55);
	_delay_ms(100);
	servo_4(87);
	_delay_ms(250);
	
	back_pick();
	_delay_ms(300);
	back_mm(10);
	small_H_slow_white();
	forward_mm(8);
	stop();
}
void centre_to_white_small_house_slow()
{
	//back_mm(0);
	stop();
	_delay_ms(0);
	servo_1(55);
	_delay_ms(100);
	servo_4(87);
	_delay_ms(100);
	
	back_pick();
	
	
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
	
}
void centre_to_white_big_house_speed()
{
	servo_4(87);
	_delay_ms(100);
	back_pick();
	
	back_mm(0);
	big_H_white_speed();
	forward_mm(0);
}
void centre_to_white_big_house_slow()
{
	servo_4(87);
	_delay_ms(100);
	back_pick();
	back_mm(35);
	big_H_speed();
	forward_mm(25);
}

/*
 * Function Name:rightmost_from_ground
 * Input: it takes "a" as a input 
 * Output: none
 * Logic: it is used to pick cm from ground and place it in rightmost part of platform
 * 
 * Example Call: it is called in pick function 
*/
void rightmost_from_ground(int a)
{
	servo_1(55);
	_delay_ms(250);
	if(a==1)
	cm_pick_1();
	else
	cm_pick_2();
	_delay_ms(500);
	str();
	_delay_ms(250);
	
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_place();
	
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
}
// these functions are use to place cms from rightmost place of platform to the respective houses
void rightmost_to_big_speed()
{
	servo_1(55);

for(int i=95;i>=0;i--)
{
	servo_4(i);
	_delay_ms(1);
}
	back_pick();
	//big_H();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	big_H_speed();
}
void rightmost_to_big_slow(){
	servo_1(55);
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	big_H_slow();
	
}
void rightmost_to_small_speed()
{
	servo_1(55);
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_speed();
	
}
void rightmost_to_small_slow()
{
	servo_1(55);
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_slow();
}
void rightmost_to_white_small_house_speed()
{
	servo_1(55);
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	//back_mm(0);
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
}
void rightmost_to_white_small_house_slow()
{
	//back_mm(45);
	stop();
	_delay_ms(100);
	servo_1(55);
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
	
}
void rightmost_to_white_big_house_speed()
{
	
	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(35);
	big_H_speed();
	forward_mm(25);
}
void rightmost_to_white_big_house_slow()
{

	for(int i=95;i>=0;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=0;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(35);
	big_H_slow();
	forward_mm(25);
}

/*
 * Function Name:right_from_ground
 * Input: it takes "a" as a input 
 * Output: none
 * Logic: it is used to pick cm from ground and place it in right part of platform
 * 
 * Example Call: it is called in pick function 
*/
void right_from_ground(int a)
{
	servo_1(55);
	_delay_ms(250);
	if(a==1)
	cm_pick_1();
	else
	cm_pick_2();
	_delay_ms(500);
	str();
	_delay_ms(250);
	
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_place();
	
	for(int i=40;i>=95;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	str();
	
}
// these functions are use to place cms from right place of platform to the respective houses
void right_to_big_speed()
{
	servo_1(55);

	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	//big_H();
	big_H_speed();
}
void right_to_big_slow(){
	servo_1(55);
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	big_H_slow();
	
}
void right_to_small_speed()
{
	servo_1(55);
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_speed();
	
}
void right_to_small_slow()
{
	servo_1(55);
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	small_H_slow();
}
void right_to_white_small_house_speed()
{
	servo_1(55);
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
}
void right_to_white_small_house_slow()
{
	//back_mm(45);
	stop();
	_delay_ms(100);
	servo_1(55);
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	
	
	back_mm(10);
	small_H_slow();
	forward_mm(8);
	stop();
}
void right_to_white_big_house_speed()
{
	
	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(35);
	big_H_speed();
	forward_mm(25);
}
void right_to_white_big_house_slow()
{

	for(int i=95;i>=40;i--)
	{
		servo_4(i);
		_delay_ms(1);
	}
	back_pick();
	for(int i=40;i<=95;i++)
	{
		servo_4(i);
		_delay_ms(10);
	}
	back_mm(35);
	big_H_slow();
	forward_mm(25);
}
void pick(int pos_cm,int time){
	if(pos_cm == 0)
	{
	   if(time==1)
	    centre_from_ground(1);
		else
		centre_from_ground(2);
	}
	else if(pos_cm == 1)
	{
	  if (time==1)
	  left_from_ground(1);
	  else
	  left_from_ground(2);
	}
	else if(pos_cm == 2)
	{
		if(time==1)
		right_from_ground(1);
		else
		right_from_ground(2);
	}
		
	else if(pos_cm == 3)
	{  
		if(time==1)
		leftmost_from_ground(1);
		else
		leftmost_from_ground(2);
	}
	else if(pos_cm == 4)
	{
	     if(time==1)
		rightmost_from_ground(1);
		else
		rightmost_from_ground(2);
    }
	
		
}
void place(int pos_cm,int height,int speed){
	if(pos_cm == 0){
		if(height == 0){
			if(speed == 0){
				centre_to_small_slow();
			}
			else if(speed ==1){
				centre_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				centre_to_big_slow();
			}
			else if(speed ==1){
				centre_to_big_speed();
			}
		}
	}
	else if(pos_cm == 1){
		if(height == 0){
			if(speed == 0){
				left_to_small_slow();
			}
			else if(speed ==1){
				left_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				left_to_big_slow();
			}
			else if(speed ==1){
				left_to_big_speed();
			}
		}
	}
	else if(pos_cm == 2){
		if(height == 0){
			if(speed == 0){
				right_to_small_slow();
			}
			else if(speed ==1){
				right_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				right_to_big_slow();
			}
			else if(speed ==1){
				right_to_big_speed();
			}
		}
	}
	else if(pos_cm == 3){
		if(height == 0){
			if(speed == 0){
				leftmost_to_small_slow();
			}
			else if(speed ==1){
				leftmost_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				leftmost_to_big_slow();
			}
			else if(speed ==1){
				leftmost_to_big_speed();
			}
		}
	}
	else if(pos_cm == 1){
		if(height == 0){
			if(speed == 0){
				left_to_small_slow();
			}
			else if(speed ==1){
				left_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				left_to_big_slow();
			}
			else if(speed ==1){
				left_to_big_speed();
			}
		}
	}
	else if(pos_cm == 4){
		if(height == 0){
			if(speed == 0){
				rightmost_to_small_slow();
			}
			else if(speed ==1){
				rightmost_to_small_speed();
			}
		}
		else if(height == 1){
			if(speed == 0){
				rightmost_to_big_slow();
			}
			else if(speed ==1){
				rightmost_to_big_speed();
			}
		}
	}

	
}
void junction_function(){
	int cm_name = -1;
	int cm_index = -1;
	int house_index = -1;
	int temp_pos = nxt_pos;
	int i,j = 0;
	ccm = 0;
	int node = cur_pos;
	declaration();
	//printf("cnt b4 calculation  =%d\n",cnt);
	if(node == 3 || node == 5||node == 7 || node == 11||node == 15 || node == 13){
		printf("cm\n");
		if(node == 3)
		cm_name = 1;
		else if(node == 5)
		cm_name = 3;
		else if(node == 7)
		cm_name = 5;
		else if(node == 11)
		cm_name = 2;
		else if(node == 13)
		cm_name = 4;
		else if(node == 15)
		cm_name = 6;
		cm_index = cm_name - 1;
		cnt = 0;
		// printf("inside node if  %d \n",cur_pos);
		for(i=0;i<5;i++)
		{
			//printf(" %d platform-%d  \n",platform[i],i);
			if(platform[i]!=-1){
				if(platform[i]==house[4][1] || platform[i]==house[4][0])
				continue;
				else
				cnt++;
			}
		}
		// printf(" %d cnt  \n",cnt);
		if(white_del == 0){
			if(left_pref == 1 && cnt <= 2){
				for(i=0;i<2;i++){
					//  printf(" 1%d houdse %d %d \n",i,house[1][i],cm_name);
					//  printf(" 3 %d houdse %d \n",i,house[3][i]);
					//  printf(" 4 %d houdse %d \n",i,house[4][i]);
					if(house[1][i]==cm_name){
						ccm++;
						
					}
					
					if(house[3][i]==cm_name){
						ccm++;
					}
					
					if(house[4][i]==cm_name){
						ccm++;
					}
				}
			}
			else if(right_pref == 1 && cnt <= 2){
				for(i=0;i<2;i++){
					//  printf(" 0 %d houdse %d %d \n",i,house[0][i],cm_name);
					//  printf(" 2 %d houdse %d \n",i,house[2][i]);
					//  printf(" 4 %d houdse %d \n",i,house[4][i]);
					if(house[0][i]==cm_name)
					ccm++;
					if(house[2][i]==cm_name)
					ccm++;
					if(house[4][i]==cm_name){
						ccm++;
					}
				}
			}
		}
		else if(white_del == 1){
			for(int i = 0; i < 4 ;i++){
				for(int j = 0; j < 2 ;j++){
					if(house[i][j] == cm_name){
						ccm++;
					}
				}
			}
			
		}
		printf(" %d cnt %d ccm  \n",cnt,ccm);
		//printf("cnt after calculation  =%d\n",cnt);
		// printf(" b4 if ppos-%d cpos_%d  npos %d temp_pos%d\n",prv_pos,cur_pos,nxt_pos,temp_pos);
		if((ccm == 1 || (ccm == 2 && cnt == 2))){
			if(cm[cm_index][0]==1)
			{
				temp_pos = nxt_pos;
				for(j=0;j<5;j++)
				{
					if(platform[j]==-1)
					{
						platform[j]=cm_name;
						//printf("Pic cm 1 and place on %d",j);
						nxt_pos = 'Q';
						grapher();
						pick(j,1);
						prv_pos = nxt_pos;
						cm[cm_index][0] = 0;
						break;
					}
				}
			}
			else{
				temp_pos = nxt_pos;
				for(j=0;j<5;j++)
				{
					if(platform[j]==-1)
					{
						platform[j]=cm_name;
						//printf("Pic cm 1 and place on %d",j);
						nxt_pos = 'I';
						grapher();
						pick(j,2);
						prv_pos = nxt_pos;
						cm[cm_index][1] = 0;
						break;
					}
				}

			}
		}
		else if(ccm == 2 && cnt  <= 1){
			temp_pos = nxt_pos;
			//printf("ppos-%d cpos_%d  npos %d temp_pos%d\n",prv_pos,cur_pos,nxt_pos,temp_pos);
			for(j=0;j<5;j++)
			{
				if(platform[j]==-1)
				{
					platform[j]=cm_name;
					//printf("Pic cm 1 and place on %d",j);
					nxt_pos = 'Q';
					grapher();
					pick(j,1);
					prv_pos = nxt_pos;
					cm[cm_index][0] = 0;
					break;
				}
			}
			// printf(" b4 2nd for loop ppos-%d cpos_%d  npos %d temp_pos%d\n",prv_pos,cur_pos,nxt_pos,temp_pos);
			for(j=0;j<5;j++)
			{
				//temp_pos = nxt_pos;
				if(platform[j]==-1)
				{
					platform[j]=cm_name;
					//printf("Pic cm 1 and place on %d",j);
					nxt_pos = 'I';
					grapher();
					pick(j,2);
					prv_pos = nxt_pos;
					cm[cm_index][1] = 0;
					break;
				}
			}
			//printf("ppos-%d cpos_%d  npos %d temp_pos%d\n",prv_pos,cur_pos,nxt_pos,temp_pos);
			

		}
		nxt_pos = temp_pos;
		//printf("after elif ppos-%d cpos_%d  npos %d temp_pos%d\n",prv_pos,cur_pos,nxt_pos,temp_pos);
		grapher();
	}
	else if((node == 4 || node == 6 ||node == 12 || node == 14)&&(white_del == 1)){
		printf("house\n");
		int hcm = 1; //placing first or second 1 if first 0 if second
		if(node == 4)
		house_index = 0;
		else if(node == 6)
		house_index = 2;
		else if(node == 12)
		house_index = 1;
		else if(node == 14)
		house_index = 3;
		int temp_pos = nxt_pos;
		for(int i =0;i<4;i++){
			for(int j = 0;j<2;j++){
				printf("platform[%d] = %d \n",i,platform[i]);
				printf("house[%d][%d] = %d\n",house_index,j,house[house_index][j]);

				if(platform[i]==house[house_index][j])
				{
					nxt_pos = 'Q';
					grapher();
					place(i,house_height[house_index],hcm);
					prv_pos = nxt_pos;
					house[house_index][j] = 0;
					platform[i] = -1;
					cnt--;
					hcm--;
					if(hcm < 0){
						break;
					}
				}
			}


		}
		nxt_pos = temp_pos;
		grapher();
	}
	else if(node == 9){
		int temp_pos = nxt_pos;
		house_index = 4;
		int hcm = 1;
		printf("%d inside node 9\n",node);
		for(int i =0;i<5;i++){
			for(int j = 0;j<2;j++){
				// printf(" house[%d][ %d] == %d \n",house_index,j,house[house_index][j]);
				//printf("%d house_index\n",house_index);
				// printf("platform[%d] = %d\n",i,platform[i]);
				if(platform[i]==house[house_index][j])
				{
					//printf("platform[%d] = %d \n",i,platform[i]);
					//printf("house[%d][%d] = %d\n",house_index,j,house[house_index][j]);
					nxt_pos = 'I';
					grapher();
					white_house(i,house_height[house_index],hcm);
					house[house_index][j] = 0;
					prv_pos = nxt_pos;
					platform[i] = -1;
					cnt--;
					hcm--;
				}
			}


		}
		white_del = 1; //indicating white house is fulfilled
		nxt_pos = temp_pos;
		grapher();
	}
	else{
		grapher();
	}
	
}
void junction_function_main(){
	while(count < len_main_path){
		count++;
		prv_pos = cur_pos;
		cur_pos = nxt_pos;
		nxt_pos = main_path[path_pos++];
		junction_function();
		//declaration();
	}
}

void waste(){
	do{
		sweep_lil();
		uart0_puts("sweep loop\n");
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
	}while((R < R_min && C > C_max && L < L_min) != 1);
	if(error_flagR == 1){
		soft_right_2();
		angle_rotate(5);
	}
	_delay_ms(25);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if((R < R_min && C > C_max && L < L_min) != 1){
		waste();
	}
	
}
  
  //below given both functins makes robot to search for black line
void sweep_lil(){
	stop();
	int limit = 70;
	velocity(lmax_speed*.5,rmax_speed*.5);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if(R < 70 && C < 70 && L < 70){
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		while(R < 100 && C < 70 && L < 100){
			int i = 0;
			while((C<70) && ( limit*1.15 > i)){
				left();
				//angle_rotate(1);
				_delay_ms(5);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				i++;
			}
			stop();
			if(C > 70 ){
				break;
			}
			int j  = 0;
			while((C < 70)  &&  limit > j){
				right();
				//angle_rotate(1);
				_delay_ms(5);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				j++;
			}
			stop();
			if(C > 70){
				break;
			}
			j  = 0;
			while((C < 70) && ( limit > j)){
				right();
				//angle_rotate(1);
				_delay_ms(5);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				j++;
			}
			stop();
			if(C > 70){
				break;
			}
			i = 0;
			while((C < 70)  && ( limit > i)){
				left();
				//angle_rotate(1);
				_delay_ms(5);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				i++;
			}
			stop();
			if(C > 70){
				break;
			}
			//limit = limit * 1.3;
		}
	}
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if((R < 70 && C < 70 && L < 70)){
		sweep_lil();
	}
	else{
		stop();
	}
}
void sweep(void){                              //line follower fns starts 
	int limit = 15;
	int swp_time = 3;
	swp_val = 50;
	velocity(rmax_speed,lmax_speed);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if(R < swp_val && C < swp_val && L < swp_val){
		//forward();
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		while(R < swp_val && C < swp_val && L < swp_val){
			stop();
			int i = 0;
			R = ADC_Conversion(1);
			C = ADC_Conversion(2);
			L = ADC_Conversion(3);
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit*1.2 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				left();
				angle_rotate(swp_time);
				
				i++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			int j  = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			j  = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			i = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				left();
				angle_rotate(swp_time);
				i++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			
		}
	}
}
 //below given  functin makes robot to search for white line in black surface
void sweep_invtd(void){
	int limit = 10;
	int swp_time = 1;
	swp_val = 200;
	velocity(rmax_speed*.5,lmax_speed*.5);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if(R > swp_val && C > swp_val && L > swp_val){
		forward();
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		while(R > swp_val && C > swp_val && L > swp_val){
			stop();
			int i = 0;
			R = ADC_Conversion(1);
			C = ADC_Conversion(2);
			L = ADC_Conversion(3);
			while((R > swp_val && C > swp_val && L > swp_val) && ( limit > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				angle_rotate(swp_time);
				
				i++;
			}
			stop();
			if(R > swp_val && C > swp_val && L > swp_val){
				;
			}
			else{
				break;
			}
			int j  = 0;
			while((R > swp_val && C > swp_val && L > swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R > swp_val && C > swp_val && L > swp_val){
				;
			}
			else{
				break;
			}
			j  = 0;
			while((R > swp_val && C > swp_val && L > swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R > swp_val && C > swp_val && L > swp_val){
				;
			}
			else{
				break;
			}
			i = 0;
			while((R > swp_val && C > swp_val && L > swp_val) && ( limit > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				left();
				angle_rotate(swp_time);
				i++;
			}
			stop();
			if(R > swp_val && C > swp_val && L > swp_val){
				;
			}
			else{
				break;
			}
			
		}
	}
}

void simple_follow(void){                              //line follower fns starts
	int limit = 15;
	int swp_time = 3;
	swp_val = 50;
	velocity(rmax_speed*.5,lmax_speed*.5);
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	if(R < swp_val && C < swp_val && L < swp_val){
		//forward();
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		while(R < swp_val && C < swp_val && L < swp_val){
			stop();
			int i = 0;
			R = ADC_Conversion(1);
			C = ADC_Conversion(2);
			L = ADC_Conversion(3);
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit*1.2 > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				left();
				angle_rotate(swp_time);
				
				i++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			int j  = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			j  = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > j)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				right();
				angle_rotate(swp_time);
				j++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			i = 0;
			while((R < swp_val && C < swp_val && L < swp_val) && ( limit > i)){
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
				velocity(rmax_speed,lmax_speed);
				left();
				angle_rotate(swp_time);
				i++;
			}
			stop();
			if(R < swp_val && C < swp_val && L < swp_val){
				;
			}
			else{
				break;
			}
			
		}
	}
}
//makes robot to follow black line
void follow_path(){
	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	path_key = 0;
// 	}
	if(R < R_mid && C > C_min && L < L_mid){
		forward();
		velocity(rmax_speed,lmax_speed);
	}
	else if(R < R_mid && C < C_mid && L > L_min){
		soft_left();
		velocity(rmax_speed,lmax_speed);
	}
	else if(R > R_min && C < C_mid && L < L_mid){
		soft_right();
		velocity(rmax_speed,lmax_speed);
	}
	else if((R > R_mid && C > C_mid)||(L > L_mid && C > C_mid)){
		velocity(rmax_speed,lmax_speed*.9);
		//_delay_ms(10);
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		if((R > R_mid && C > C_mid)||(L > L_mid && C > C_mid)){
			stop();
			
// 						ShaftCountLeft = 0;
// 						ShaftCountRight = 0;
						forward_mm(110);
						stop();
						_delay_ms(100);
						//left_turn();
						//enter_invtd();
						//stop();
						//_delay_ms(5000);
			//junction_function();
			junction_function_main();
 			
		}
	}
	
	else{
		int k = 0;
		while( (R < R_min && C < C_min && L < L_min) && k < max_tym){
				_delay_ms(1);
				R = ADC_Conversion(1);
				C = ADC_Conversion(2);
				L = ADC_Conversion(3);
			    k++;
			}
		if (k >= max_tym){
			sweep();
		}
	}
}
// makes robot to follow white line in black path
void inverted_path(){
	int i_min = 30;
	int iR_max = 200;
	int iL_max = 200;
	int iC_max = 200;

	R = ADC_Conversion(1);
	C = ADC_Conversion(2);
	L = ADC_Conversion(3);
	forward();
	if(((R < R_mid && C > C_min && L < L_mid) || (R > i_min && C < C_mid && L < L_min ) || (R < R_min && C < C_mid && L > L_min)) && (ijunction == 1))
	{
		path_key = 0;
		follow_path();
	}
	else if(R > iR_max && C < i_min && L > iL_max){
		forward();
		velocity(rmax_speed,lmax_speed);
	}
	else if(R > iR_max && C > iC_max && L < i_min){
		soft_left();
		velocity(rmax_speed*.5,lmax_speed*.5);
	}
	else if(R < i_min && C > iC_max && L > iL_max){
		soft_right();
		velocity(rmax_speed*.5,lmax_speed*.5);
	}
	
	
	else if(((R < 13 && C < 70 && L > 150) || (L < 13 && C < 70 && R > 150) || (R < 100 && C < 30 && L < 30) || (L < 100 && C < 30 && R < 30)|| (R < 15 && R < 15)|| (L < 15 && C < 15))&& (ijunction == 0)){
		velocity(rmax_speed*7,lmax_speed*.7);
		forward();
		_delay_ms(20);
		R = ADC_Conversion(1);
		C = ADC_Conversion(2);
		L = ADC_Conversion(3);
		//if((R < i_min && C < i_min)||(L < i_min && C < i_min)||
		//if((R < 30 && C < 200 && L > 200) || (L < 30 && C < 200 && R > 200)|| (R < 15 && C < 15) || (L < 15 && R < 15)){
			//junction_function();
			//int node_time = 250;
			stop();
			ijunction = 1;
			forward_mm(100);
			stop();
			junction_function_main();
			//right();
		//}
	}
	
	else{
		int k = 0;
		int max_tym1 = 10;
		while( (R > iR_max && C > iC_max && L > iL_max) && k < max_tym1){
			_delay_ms(1);
			R = ADC_Conversion(1);
			C = ADC_Conversion(2);
			L = ADC_Conversion(3);
			k++;
		}
		if (k >= max_tym1){
			sweep_invtd();
		}
	}
}
// function gets called when robot enters black surface
void enter_invtd(){
	left();
	angle_rotate(10);
	_delay_ms(80);
	forward_mm(30);
	path_key = 1;
	ijunction = 0;
}

int main()
{
	init_devices();
	uart0_init(UART_BAUD_SELECT(115200,(F_CPU)));
	uart0_flush();
	declaration();
	path_planner();
	//servo_1(130);
	servo_1(55);                      // plzz dont remove this three lines add this always at start of code
	_delay_ms(100);
	str();
	
    path_key = 0;
	ijunction = 0;	
	prv_pos = 0;
	cur_pos = 0;
	nxt_pos = 1; ///
	path_pos = 2;
	while(1){
		if(path_key == 0){
			follow_path();
		}
		else{			
			inverted_path();
		}
		
		if(count == 26){
			buzzer_on();
			_delay_ms(4000);
			buzzer_off();
			break;
		}
 	}
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	_delay_ms(2000);
	
	
	servo_1_free();
	servo_2_free();
	servo_3_free();
	servo_4_free();








 
}
