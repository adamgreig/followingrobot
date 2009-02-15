/************************************************
 *	FollowingRobot - lcd.h
 *	Functions to interact with an LCD
 *	screen connected to SPI1.
 *	Header file.
 *	See README for more details.
 ***********************************************/

#include "stm32f10x_lib.h"

#define LCD_ADDR 0x74

//Set a register in LCD RAM
void _lcd_write_reg( u8 index, u16 data );

//Initialise the LCD
void lcd_initialise();

//Start and finish sending data to the LCD
void lcd_startdata();
void lcd_enddata();

//Prototype for Delay, defined in main.c, a busy wait loop
void Delay( unsigned long delay );
