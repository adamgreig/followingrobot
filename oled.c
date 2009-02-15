/************************************************
 *	FollowingRobot - oled.c
 *	Functions to interact with an OLED
 *	screen connected to USART1.
 *	Implementation file.
 *	See README for more details.
 ***********************************************/

#include "oled.h"

void _usart_send_data( char* data, int len ) {
	int i;
	for( i = 0; i < len; i++ ) {
		USART_SendData( USART, data[i] );
		while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}
	}
};

void oled_wait_for_ack() {
	while( USART_GetFlagStatus(USART, USART_FLAG_RXNE) == RESET ) {}
	u16 rxData = USART_ReceiveData( USART );
	if( rxData != 0x06 ) oled_set_background_colour( 0xF800 );
};

void oled_autobaudrate() {
	char data[] = {0x55};
	_usart_send_data( data, 1 );
};

void oled_set_background_colour( u16 colour ) {
	char data[] = {'B', colour>>8, colour};
	_usart_send_data( data, 3 );

	//special case: obviously can't use normal wait_for_ack
	// as it sets the background colour using this function

	while( USART_GetFlagStatus( USART, USART_FLAG_RXNE ) == RESET ) {}
	USART_ReceiveData( USART );
};

void oled_erase_screen() {
	char data[] = {'E'};
	_usart_send_data( data, 1 );

	oled_wait_for_ack();
};

void oled_font_size( u8 font_size ) {
	char data[] = {'F', font_size};
	_usart_send_data( data, 2 );

	oled_wait_for_ack();
};

void oled_rectangle( u8 x1, u8 y1, u8 x2, u8 y2, u16 colour ) {
	char data[] = {'r', x1, y1, x2, y2, colour>>8, colour};
	_usart_send_data( data, 7 );

	oled_wait_for_ack();
};

void oled_formatted_string( u8 column, u8 row, u8 font_size, u16 colour, char* str ) {
	int len;
	for( len=0; str[len] != 0x00; len++ ) {};

	char data[7+len];
	data[0] = 's';
	data[1] = column;
	data[2] = row;
	data[3] = font_size;
	data[4] = colour>>8;
	data[5] = colour;

	int i;
	for( i = 1; i <= len; i++ ) {
		data[5+i] = str[i - 1];
	}
	data[7+len - 1] = 0x00;

	_usart_send_data( data, 7+len );

	oled_wait_for_ack();

};

/*
void oled_image( u8 x, u8 y, u8 width, u8 height, u8 colourmode, u16** data ) {
	//send over the serial port ourselves
	USART_SendData( USART, 'I' );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	USART_SendData( USART, x );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	USART_SendData( USART, y );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	USART_SendData( USART, width );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	USART_SendData( USART, height );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	USART_SendData( USART, colourmode );
	while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}

	u8 cx;
	u8 cy;
	u8 buf;
	for( cy=0; cy<height; cy++ ) {
		for( cx=0; cx<width; cx++ ) {
			//this code assumes colourmode=16 since the data from the camera will be, and reverse the bit direction
			buf = (data[cy][cx] & 0x1F) | ((data[cy][cx] & 0x700)>>8);
			USART_SendData( USART, buf );
			while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}
			buf = (data[cy][cx] & 0xE0) | ((data[cy][cx] & 0xF800)>>3);
			USART_SendData( USART, buf );
			while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {}
		}
	}
	oled_wait_for_ack();
};
*/

void oled_power(int state) {
	char data[] = {'Y', 0x03, state};
	_usart_send_data( data, 3 );
	oled_wait_for_ack();
};

