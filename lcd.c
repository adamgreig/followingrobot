/************************************************
 *	FollowingRobot - lcd.c
 *	Functions to interact with an LCD
 *	screen connected to SPI1.
 *	Implementation file.
 *	See README for more details.
 ***********************************************/

 #include "lcd.h"

void _lcd_write_reg( u8 index, u16 data ) {

	// Send the index and wait for transmit to be complete
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_RESET );

	SPI_I2S_SendData( SPI1, LCD_ADDR );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {}

	SPI_I2S_SendData( SPI1, 0x00 );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_I2S_SendData( SPI1, index );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_SET );

	// Send the data and wait for transmit to be complete
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_RESET );

	SPI_I2S_SendData( SPI1, LCD_ADDR | 0x02  );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_I2S_SendData( SPI1, (u8)(data>>8) );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_I2S_SendData( SPI1, (u8)data );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_SET );

}

void lcd_initialise() {

	// This intense series of commands sets up a lot of registers in the LCD, to
	//  enable internal voltage regulators and boosters, start the drive circuitry,
	//  configure contrast levels and colour balances, configure inputs and masks,
	//  set up the window size of incoming data and basically gets the screen ready
	//  to receive data. If this is all done correctly the screen will go black opaque.

	_lcd_write_reg( 0x00, 0x0001 );
	Delay( 20 * 20000 );

	_lcd_write_reg( 0x08, 0x0303 );
	_lcd_write_reg( 0x02, 0x0500 );
	_lcd_write_reg( 0x01, 0x010B );
	_lcd_write_reg( 0x0B, 0x4001 );
	_lcd_write_reg( 0x30, 0x0000 );
	_lcd_write_reg( 0x31, 0x0000 );
	_lcd_write_reg( 0x32, 0x0000 );
	_lcd_write_reg( 0x33, 0x0401 );
	_lcd_write_reg( 0x34, 0x0707 );
	_lcd_write_reg( 0x35, 0x0707 );
	_lcd_write_reg( 0x36, 0x0707 );
	_lcd_write_reg( 0x37, 0x0104 );
	_lcd_write_reg( 0x38, 0x0004 );
	_lcd_write_reg( 0x39, 0x0004 );
	_lcd_write_reg( 0x41, 0x0280 );
	_lcd_write_reg( 0x42, 0x8300 );
	_lcd_write_reg( 0x43, 0x9F9F );
	_lcd_write_reg( 0x11, 0x0001 );
	_lcd_write_reg( 0x12, 0x0008 );
	_lcd_write_reg( 0x13, 0x100E );
	_lcd_write_reg( 0x10, 0x0044 );
	_lcd_write_reg( 0x12, 0x0018 );
	_lcd_write_reg( 0x40, 0x0000 );
	_lcd_write_reg( 0x41, 0x0000 );
	_lcd_write_reg( 0x42, 0x5F00 );

	Delay( 40 * 20000 );

	_lcd_write_reg( 0x13, 0x300C );

	Delay( 60 * 20000 );

	_lcd_write_reg( 0x10, 0x4340 );

	Delay( 100 * 20000 );

	_lcd_write_reg( 0x21, 0x0004 );
	_lcd_write_reg( 0x44, 0x8304 );
	_lcd_write_reg( 0x45, 0x7F00 );
	_lcd_write_reg( 0x07, 0x0205 );

	Delay( 40 * 20000 );

	_lcd_write_reg( 0x07, 0x0227 );

	Delay( 1 * 20000 );

	_lcd_write_reg( 0x03, 0x1030 );
	_lcd_write_reg( 0x0D, 0x3336 );
	_lcd_write_reg( 0x07, 0x1237 );

	Delay( 10 * 20000 );

	_lcd_write_reg( 0x44, 0x7F00 );
	_lcd_write_reg( 0x45, 0x5F00 );
	_lcd_write_reg( 0x23, 0x0000 );
	_lcd_write_reg( 0x24, 0x0000 );
	_lcd_write_reg( 0x21, 0x0000 );

	Delay( 10 * 20000 );
}

void lcd_startdata() {
	// Prepare the LCD to receive all the data
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_RESET );

	SPI_I2S_SendData( SPI1, LCD_ADDR );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_I2S_SendData( SPI1, 0x00 );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	SPI_I2S_SendData( SPI1, 0x22 );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_SET );

	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_RESET );

	SPI_I2S_SendData( SPI1, LCD_ADDR | 0x02 );
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
}

void lcd_enddata() {
	// Finished sending data
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_SET );
}
