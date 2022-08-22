/*
 * Microchip PIC-based Bench Power Supply
 *
 * This is the firmware for a DIY bench power supply based on a Microchip PIC16F886.
 * 
 */

#include <htc.h>
#include <string.h>
#include "benchpower.h"
#include "lcd.h"

__CONFIG (DEBUG_OFF & LVP_OFF & FCMEN_OFF & IESO_OFF & CPD_OFF & CP_OFF & \
	MCLRE_ON & PWRTE_ON & WDTE_OFF & BOREN_OFF & FOSC_HS);
__CONFIG (WRT_OFF & BOR4V_BOR40V);

/* this is the maximum value of an A2D conversion. */
#define _XTAL_FREQ 20000000

#define MAX_VOLTS 12000
#define MAX_AMPS 1000

unsigned char sw, sw_last, new_param;
unsigned char sw1_mode, sw2_mode, ps_mode;
unsigned char delay1, delay2;
char Kp, Kd, Ki;
int pwmduty;
int i_select, i_ad, i_limit, i_last;
int v_select, v_ad, v_target, v_last;
int err, perr, derr, ierr;

void init(void){
	OSCCON=0x70;			// Set oscillator to 8 MHz (Using 20MHz OSC)
	ANSEL=0x03;				// RA<0:1> as analog
	ANSELH=0;
	ADCON1=0x80;			// right justified
	ADCON0=0x81;			// ADC clock Fosc/32, select AN0, enable ADC
	ADIE=0;					// not interrupt driven
	ADON=1;					// enable A2D converter
	TRISA=0x03;				// RA<0:1> as input
	TRISB=0xff;				// Set PORTB in input mode
	TRISC=0x00;				// Set PORTC in output mode
	TRISE=0x00;				// Set PORTE in output mode
	OPTION_REG = 0b0011;	// TMR0 prescale by 16 (976Hz or ~1ms)
	T0CS=0;					// select internal clock
	T0IE=1;					// enable timer 0 interrupt
	PR2=0x7f;				// Set PWM frequency (39kHz)
	T2CON=0x04;				// Turn on TMR2
	TMR2IE=1;				// enable timer 2 interrupt
	CCPR1L=0;				// Set PWM duty to 0
	CCP1CON=0x0c;			// enable PWM
}

void setpwmduty(int d){
	CCP1CONbits.DC1B=(d&0x03);
	CCPR1L=d>>2;
}

char* bin2bcd( unsigned int n, char *s ){

	unsigned int d4, d3, d2, d1, d0, q;

	d1 = (n>>4)  & 0xF;
	d2 = (n>>8)  & 0xF;
	d3 = (n>>12) & 0xF;

	d0 = 6*(d3 + d2 + d1) + (n & 0xF);
	q = (d0 * 0xCD) >> 11;
	d0 = d0 - (q<<3) - (q<<1);

	d1 = q + 9*d3 + 5*d2 + d1;
	q = (d1 * 0xCD) >> 11;
	d1 = d1 - (q<<3) - (q<<1);

	d2 = q + 2*d2;
	q = (d2 * 0x1A) >> 8;
	d2 = d2 - (q<<3) - (q<<1);

	d3 = q + 4*d3;
	d4 = (d3 * 0x1A) >> 8;
	d3 = d3 - (d4<<3) - (d4<<1);

	s[0]=d4+'0';
	s[1]=d3+'0';
	s[2]=d2+'0';
	s[3]=d1+'0';
	s[4]=d0+'0';

	return s;
}

unsigned int bin2volt(unsigned int n){
	return ((n<<3)+(n<<2)-(n>>4));
}

unsigned int bin2amp(unsigned int n){
	return (((n<<5)+(n<<3)-n)>>3);
}

unsigned int volt2bin(unsigned int n){
	return (((n+(n>>2)+(n>>4)+(n>>5))>>4));
}

unsigned int amp2bin(unsigned int n){
	return (((n<<2)-(n>>1)-(n>>2)+(n>>6))>>4);
}

void interrupt isr(void){

	if(T0IE && T0IF){
		sw=PORTB;
		if(sw!=sw_last) {
			switch(((sw&0b11)<<2) | (sw_last&0b11)){
				case 0b0111:
					if(sw1_mode==0) {
						v_select-=100;
						if(v_select<0) v_select=0;
					} else {
						i_select-=10;
						if(i_select<0) i_select=0;
					}
					break;
				case 0b1011:
					if(sw1_mode==0) {
						v_select+=100;
						if(v_select>MAX_VOLTS) v_select=MAX_VOLTS;
					} else {
						i_select+=10;
						if(i_select>MAX_AMPS) i_select=MAX_AMPS;
					}
			}
			switch((sw&0b1100) | ((sw_last>>2)&0b11)){
				case 0b0111:
					switch(sw2_mode) {
					case 0: Kp--; break;
					case 1: Kd--; break;
					case 2: Ki--;
					}
					break;
				case 0b1011:
					switch(sw2_mode) {
					case 0: Kp++; break;
					case 1: Kd++; break;
					case 2: Ki++;
					}
			}
			if((sw&0x10)==0 && ((sw_last&0x10)==0x10)) {
				if(++sw1_mode==2) sw1_mode=0;
				if(++sw2_mode==3) sw2_mode=0;
			}
			if((sw&0x20)==0 && ((sw_last&0x20)==0x20)) {
				if(ps_mode==0) ps_mode=2;
				else ps_mode=0;
			}
			sw_last=sw;
			new_param=1;
		}
		T0IF=0;
	}

	if(TMR2IE && TMR2IF) {
		if(delay1==0) {

			//Sample AD

			PORTAbits.RA2=1;
			PORTAbits.RA3=1;
			ADCON0bits.CHS=1;
			__delay_us(5);
			GO=1;
			while(GO)continue;
			ADIF=0;
			v_ad=(ADRESH&0b11)<<8|ADRESL;
			PORTAbits.RA2=0;

			ADCON0bits.CHS=0;
			__delay_us(5);
			GO=1;
			while(GO)continue;
			ADIF=0;
			i_ad=(ADRESH&0b11)<<8|ADRESL;

			if(ps_mode!=0){
				if(i_ad>i_limit) ps_mode = 2; //CC
				else if(v_ad>v_target) ps_mode = 1; //CV
			}

			switch(ps_mode) {
			case 0:
				perr=0;
				derr=0;
				ierr=0;
				break;
			case 1:
				// CV mode
				perr=(v_target-v_ad)*Kp;
				derr=(v_last-v_ad)*Kd;
				v_last=v_ad;

				if(delay2==0) {
					ierr+=(v_target-v_ad);
					if(ierr>0x7f) ierr=0x7f;
				}
				break;
			case 2:
				// CC mode
				perr=(i_limit-i_ad)*Kp;
				derr=(i_last-i_ad)*Kd;
				i_last=i_ad;

				if(delay2==0) {
					ierr+=(i_limit-i_ad);
					if(ierr>0x7f) ierr=0x7f;
				}
				break;
			}
			pwmduty=(perr+derr+ierr)>>1;
			if(pwmduty<0) pwmduty=0;
			if(pwmduty>0x1ff && pwmduty<0x4000) pwmduty=0x1ff;
			// Set PWM duty
			CCP1CONbits.DC1B=(pwmduty&0x03);
			CCPR1L=pwmduty>>2;


			if(delay2==0) delay2=Ki;
			delay2--;
			delay1=6;
			PORTAbits.RA3=0;
		}
		delay1--;
		TMR2IF=0;
	}
}

void main(void){

	unsigned char buf[5];

	init();
	lcd_init();
	lcd_puts("Benchpower");
	__delay_ms(1000);

	pwmduty=0;
	v_select=0;
	v_target=0;
	v_last=0;
	i_select=200;
	new_param=1;
	Kp=8;
	Kd=1;
	Ki=4;
	ierr=0;
	delay1=6;
	delay2=Ki;

	sw1_mode=0;	//0 = V adjust, 1 = A adjust
	sw2_mode=0;	//0 = Kp, 1 = Kd, 2 = Ki

	ps_mode=1;	// CV mode

	CCP1CON=0x0c;	// enable pwm
	PEIE=1;		// enable peripheral interrupts
	GIE=1;		// turn on interrupts

	while(1){

		if(new_param) {
			i_limit=amp2bin(i_select);
			v_target=volt2bin(v_select);

			// 1st line
			bin2bcd(i_select,buf);
			LCDText[4]=buf[1];
			LCDText[5]='.';
			LCDText[6]=buf[2];
			LCDText[7]=buf[3];
			LCDText[8]='A';
			if(sw1_mode==0) {
				LCDText[3]=' ';
				LCDText[9]=' ';
			} else {
				LCDText[3]='[';
				LCDText[9]=']';
			}
			// 2nd line
			bin2bcd(v_select,buf);
			LCDText[36]=(buf[0]=='0' ? ' ' : buf[0]);
			LCDText[37]=buf[1];
			LCDText[38]='.';
			LCDText[39]=buf[2];
			LCDText[40]='V';
			if(sw1_mode==0) {
				LCDText[35]='[';
				LCDText[41]=']';
			} else {
				LCDText[35]=' ';
				LCDText[41]=' ';
			}

			// 3rd line
			LCDText[16]='V';
			bin2bcd(v_target,&LCDText[18]);

			// 4th line
			LCDText[48]='P';
			bin2bcd(Kp,buf);
			LCDText[49]=buf[2];
			LCDText[50]=buf[3];
			LCDText[51]=buf[4];

			LCDText[53]='D';
			bin2bcd(Kd,buf);
			LCDText[54]=buf[2];
			LCDText[55]=buf[3];
			LCDText[56]=buf[4];

			LCDText[58]='I';
			bin2bcd(Ki,buf);
			LCDText[59]=buf[2];
			LCDText[60]=buf[3];
			LCDText[61]=buf[4];

			LCDText[63]=sw2_mode+'0';
		}

		// line 1
		LCDText[0]=0x30+((pwmduty>>8)&0x01);
		LCDText[1]=0x30+((pwmduty>>4)&0x0f);	// 10's digit
		if(LCDText[1]>'9') LCDText[1]+=0x7;
		LCDText[2]=0x30+(pwmduty&0x0f);			// 1's digit
		if(LCDText[2]>'9') LCDText[2]+=0x7;

		bin2bcd(bin2amp(i_ad),buf);
		LCDText[11]=buf[1];
		LCDText[12]='.';
		LCDText[13]=buf[2];
		LCDText[14]=buf[3];
		LCDText[15]=buf[4];

		// line 2
		switch(ps_mode) {
			case 0: memcpy(&LCDText[32], "OFF", 3); break;
			case 1: memcpy(&LCDText[32], "CV ", 3); break;
			case 2: memcpy(&LCDText[32], "CC ", 3);
		}

		bin2bcd(bin2volt(v_ad),buf);
		LCDText[43]=(buf[0]=='0' ? ' ' : buf[0]);
		LCDText[44]=buf[1];
		LCDText[45]='.';
		LCDText[46]=buf[2];
		LCDText[47]=buf[3];

		// line 3
		bin2bcd(v_ad,&LCDText[24]);
		lcd_update();
	}
}
