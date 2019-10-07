/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 11/19/2017
Author  : 
Company : 
Comments: 


Chip type               : ATmega32A
Program type            : Application
AVR Core Clock frequency: 1.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <mega32a.h>

// Alphanumeric LCD functions
#include <alcd.h>
#include <delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Declare your global variables here
#define dt 0.01;
#define kp 0.08;
#define ki 0.01;
#define kd 0.01;
#define epsilon 0.01;

#define R1 PORTD.0
#define L1 PORTD.1 


#define trig_pin PORTB.0
#define echo_pin PINB.1
#define k PINC.5
float last_error,last_iterm,error,iterm,dterm,pterm,output=0;
//float pterm,error=0;
float power=50;
int timer;
int d = 0;int di;
char str[20];

// Timer 0 overflow interrupt service routine
//TIM0_OVF = Timer/Counter0 Overflow
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Place your code here
 
    TCNT0=0;
  //timer is the number of overflows
    timer=1+timer;
}
int distance(){
 
       PORTB.0=1;
       delay_us(20);PORTB.0=0;
         while(PINB.1==0);
	//TCNT0 indicates the pulse frequency (125KHz)
 timer=0;TCNT0=0;TCCR0=0x02;
 while(PINB.1 == 1);
 TCCR0=0x00; //No clock source - Deactivating TC0
 
//no object detected after 30ms
 if(timer*256.0+TCNT0>30000.0){
 lcd_clear();
 lcd_gotoxy(0,0);
 lcd_puts("chizi nist");}
 else
 {
  d=0;
  d=((timer*256.0+ TCNT0)*.17);
  di=d;
  lcd_clear();
  sprintf(str,"  d=%d cm",d);
  lcd_puts(str);
  delay_ms(150);
  
      } 
      return di;
}


//motor Function
// f | F : forward
// b | B : backward
// r | R :right
// l | L :left
void motor(char direct,float power)
{
    if(direct=='f' | direct=='F')
        {
        R1=1;L1=0;
        }
        
    if(direct=='b' | direct=='B')
        {
        R1=0;L1=1;
        }
        
OCR1BH=0x00;        
//OCR1BL=(pow*255/100);
OCR1BL=0X00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1AL=power;
}
void motor_break()
{
R1=L1=0;
}

void main(void)
{ int r=35; // r is our reference distance
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (1<<DDA4) | (1<<DDA3) | (1<<DDA2) | (1<<DDA1) | (1<<DDA0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=Out 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=0 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=T Bit6=T Bit5=0 Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFF
// OC0 output: Disconnected
// Timer Period: 0.256 ms
//TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (1<<CS00);
TCCR0=0x02;
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Ph. correct PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.51 ms
// Output Pulse(s):
// OC1A Period: 0.51 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
MCUCSR=(0<<ISC2);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTA Bit 0
// RD - PORTA Bit 1
// EN - PORTA Bit 2
// D4 - PORTA Bit 4
// D5 - PORTA Bit 5
// D6 - PORTA Bit 6
// D7 - PORTA Bit 7
// Characters/line: 8
lcd_init(32);

// Global enable interrupts
#asm("sei")

while (1)
      { 
      d=distance();
//      error = r-d;
//      pterm = error*0.1;  
//      iterm = (error*0.01)+last_iterm;  
//     // dterm = (error-last_error)*kd;
//      output = (pterm+iterm)*1.5;  //2 
//     // last_error=error;
//      last_iterm=iterm; 
//      if(error>=5) {   
//      OCR1AL=100; //Controller Bias
//      OCR1AL=OCR1AL+output;
//      power=OCR1AL;
//      motor('f',power);
//      delay_ms(10); }
//      if(error<=-5){
//      OCR1AL=100; //Controller Bias
//      //pterm=-pterm;  
//      output=-output;
//      OCR1AL=OCR1AL+output; 
//      power=OCR1AL;
//      motor('b',power);
//      delay_ms(10);}
//      //if(-5<error<5){
//      //motor_break();
//      //power=0;
//      //motor('f',power);
//      //delay_ms(10);}
        if(d>5 && d<15)
        {power= 150;
        motor('f',power);
        }
        if(d>15 && d<25)
        {power=100;
        motor('f',power);}
        if(d>25 && d<35)
        {power=50;
        motor('f',power);}
        if(d>35 && d<45)
        {power= 0;
        motor('b',power);}
        if(d>45 && d<55)
        {power= 50;
        motor('b',power);}
        if(d>55 && d<65)
        {power=100;
        motor('b',power);}
        if(d>65 && d<75)
        {power=150;
        motor('b',power);}
        if(d>75 && d<90)
        {power=200;
        motor('b',power);}
        if(d>90)
        {power=250;
        motor('b',power);}

      }   }