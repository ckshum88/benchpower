/*
 *	LCD interface example
 *	Uses routines from delay.c
 *	This code will interface to a standard LCD controller
 *	like the Hitachi HD44780. It uses it in 4 bit mode, with
 *	the hardware connected as follows (the standard 14 pin 
 *	LCD connector is used):
 *	
 *	PORTD bits 0-3 are connected to the LCD data bits 4-7 (high nibble)
 *	PORTA bit 3 is connected to the LCD RS input (register select)
 *	PORTA bit 1 is connected to the LCD EN bit (enable)
 *	
 *	To use these routines, set up the port I/O (TRISA, TRISD) then
 *	call lcd_init(), then other routines as required.
 *	
 */

#define _XTAL_FREQ 20000000

#include	<htc.h>
#include	"lcd.h"

#define	LCD_RS RC0
#define	LCD_RW RC1
#define LCD_EN RC3

#define LCD_DATA	PORTC

#define	LCD_STROBE()	(__delay_us(1),(LCD_EN=1),__delay_us(1),(LCD_EN=0))

char LCDText[16*4+1];

void
lcd_update()
{
	char i;

	LCD_RS = 0;
	lcd_write(0x2);
	__delay_us(80);
	LCD_RS = 1;
	for(i = 0; i<32; i++) {
		lcd_write( LCDText[i] );
	}

	lcd_goto(0x10);
	__delay_us(80);
	LCD_RS = 1;
	for(i = 32; i<64; i++) {
		lcd_write( LCDText[i] );
	}
}

/* write a byte to the LCD in 4 bit mode */

void
lcd_write(unsigned char c)
{
	__delay_us(40);
	LCD_DATA = (LCD_DATA & 0x0f) | (c&0xf0);
	LCD_STROBE();
	LCD_DATA = (LCD_DATA&0xf) | (c<<4);
	LCD_STROBE();
}

/*
 * 	Clear and home the LCD
 */

void
lcd_clear(void)
{
	LCD_RS = 0;
	lcd_write(0x1);
	__delay_ms(2);
}

/* write a string of chars to the LCD */

void
lcd_puts(const char * s)
{
	LCD_RS = 1;	// write characters
	while(*s)
		lcd_write(*s++);
}

/* write one character to the LCD */

void
lcd_putch(char c)
{
	LCD_RS = 1;	// write characters
	lcd_write( c );
}


/*
 * Go to the specified position
 */

void
lcd_goto(unsigned char pos)
{
	LCD_RS = 0;
	lcd_write(0x80+pos);
}
	
/* initialise the LCD - put into 4 bit mode */
void
lcd_init()
{
	unsigned char i;

	for(i=0;i<sizeof(LCDText)-1;i++) {
		LCDText[i]=' ';
	}
	LCDText[sizeof(LCDText)-1] = 0;

	LCD_RS = 0;
	LCD_EN = 0;
	LCD_RW = 0;
	
	__delay_ms(40);	// wait 15mSec after power applied,
	LCD_DATA = 0b00100000;	// Four bit mode
	LCD_STROBE();
	LCD_STROBE();
	__delay_us(100);
	LCD_STROBE();
	LCD_STROBE();
	__delay_us(100);

	lcd_write(0xc); // Display On, Cursor Off, Cursor Blink Off
	__delay_us(100);
	lcd_clear();	// Clear screen
	__delay_ms(10);
	lcd_write(0x6); // Set entry Mode
	__delay_us(100);

}
