/************************************************
 *	FollowingRobot - oled.h
 *	Functions to interact with an OLED
 *	screen connected to USART1.
 *	Header file.
 *	See README for more details.
 ***********************************************/

#include "stm32f10x_lib.h"

//Colour codes in RGB565
#define BLACK	0x0000
#define RED	0xF800
#define YELLOW	0xFFE0
#define GREEN	0x07E0
#define CYAN	0x07FF
#define BLUE	0x001F
#define PURPLE	0xF81F
#define WHITE	0xFFFF

//Which USART to use
#define USART USART1

//Send an array of data over the USART
void _usart_send_data( char* data, int len );

//Busy wait for an ACK from the OLED
void oled_wait_for_ack();

//Send a 'U' (01010101) character to get the OLED determine baud rate
void oled_autobaudrate();

//Set background colour of the OLED. Used when erasing the screen.
void oled_set_background_colour( u16 colour );

//Erase the OLED screen, filling with the background colour.
void oled_erase_screen();

//Set the font size to be used by the formatted_string command.
void oled_font_size( u8 font_size );

//Draw a filled rectangle
void oled_rectangle( u8 x1, u8 y1, u8 x2, u8 y2, u16 colour );

//Write a string to the OLED screen
void oled_formatted_string( u8 column, u8 row, u8 font_size, u16 colour, char* str );

//Draw image data. Not currently used.
//void oled_image( u8 x, u8 y, u8 width, u8 height, u8 colourmode, u16** data );

//Power-toggle the OLED. This is the 'correct' way to shut it down.
void oled_power( int state );
