/************************************************
 *	FollowingRobot - main.c
 *	The main code.
 *	See README for more details.
 ***********************************************/

// The main STM32 library
#include "stm32f10x_lib.h"

//Control functions for the OLED and LCD screens
#include "lcd.h"
#include "oled.h"

//Control functions for the camera
#include "cam.h"

//Frame settings
#define FRAME_WIDTH 128
#define FRAME_HEIGHT 1

//Shortcut defines for toggling the interrupts
#define EXTI_ENABLE_HD EXTI_Init( &EXTI_InitStructure_HD );
#define EXTI_ENABLE_VD EXTI_Init( &EXTI_InitStructure_VD );

#define EXTI_DISABLE EXTI_DeInit();

//Function prototypes
void Clock_Config();	// Starts the HSE, clocks the system and enables peripheral clocks
void GPIO_Config();	// Configures all the input/output pins
void USART_Config();	// Configures USART1 at 28800 baud for the OLED
void I2C_Config();	// Configures I2C1 at clock 100kHz for the camera control
void TIM_Config();	// Configures TIM1 CH1 as a PWM output at 6MHz for the camera
void NVIC_Config();	// Configures the interrupt that fires on event lines
void EXTI_Config();	// Configures the event lines to fire on camera's sync lines
void SPI_Config();	// Configures SPI1 at 18MHz for the LCD
void DMA_Config();	// Configures DMA channel 3 for sending data to SPI1

int main();		// Code entry point

// Wait a set number of iterations, used as a very rough delay (busy wait loop)
void Delay( unsigned long delay );
// Flash the LEDs on the front ('eyes') as an indicator
void flash_eyes( int n );

// Variables for initialising peripherals
// Stores configuration options that are then applied
// to a specific peripheral.
GPIO_InitTypeDef	GPIO_InitStructure;
USART_InitTypeDef	USART_InitStructure;
I2C_InitTypeDef		I2C_InitStructure;
SPI_InitTypeDef		SPI_InitStructure;
DMA_InitTypeDef		DMA_InitStructure;
TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
TIM_OCInitTypeDef	TIM_OCInitStructure;
NVIC_InitTypeDef	NVIC_InitStructure;
EXTI_InitTypeDef	EXTI_InitStructure_HD;
EXTI_InitTypeDef	EXTI_InitStructure_VD;

//Hold clock startup error status
ErrorStatus HSEStartUpStatus;

// Global variables for the interrupt handler to use
volatile u8 linenum;	//Store current line number
volatile u8 data[ 256 ];//Store the actual line data
volatile u8 *dataptr;	//A pointer to the line data
volatile u32 numred;	//Store number of red pixels
volatile u32 sumredx;	//Store sum of x-values for red pixels
volatile u32 *numredptr;//A pointer to the number of red pixels
volatile u32 *sumredxptr;//A pointer to the sum of the x-values

int main() {

	// Configure clock
	Clock_Config();

	// Configure peripherals
	GPIO_Config();
	USART_Config();
	I2C_Config();
	TIM_Config();
	NVIC_Config();
	EXTI_Config();
	DMA_Config();
	SPI_Config();

	// Start up the OLED
	Delay( 4000000 );
	oled_autobaudrate();
	Delay( 100000 );
	oled_set_background_colour( BLACK );
	oled_erase_screen();
	oled_formatted_string( 1, 0, 0, WHITE, "Initialising..." );

	// Configure the LCD
	GPIO_WriteBit( GPIOA, GPIO_Pin_2, Bit_SET );
	lcd_initialise();
	GPIO_WriteBit( GPIOA, GPIO_Pin_2, Bit_RESET );

	//Start the camera sending data
	cam_init();

	// Flash the eyes to indicate readiness
	flash_eyes(2);

	// Set the pointers to the red data tracking variables
	numredptr	= &numred;
	sumredxptr	= &sumredx;

	//slow the clock right down (4MHz EXTCLK --> 2MHz DCLK)

	TIM_CtrlPWMOutputs( TIM1, DISABLE );
	TIM_Cmd( TIM1, DISABLE );

	TIM_TimeBaseStructure.TIM_Period	= 17;
	TIM_TimeBaseInit( TIM1, &TIM_TimeBaseStructure );

	TIM_OCInitStructure.TIM_Pulse		= 9;
	TIM_OC1Init( TIM1, &TIM_OCInitStructure );

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	oled_erase_screen();
	oled_formatted_string( 1, 0, 0, WHITE, "Receiving Data" );

	int servo_send_pulse = 10;

	// Main loop
	for(;;) {
		
		// Initialise the line count and frame pointer
		linenum		= 0;
		numred		= 0;
		sumredx		= 0;
		dataptr		= data;
		
		// Prepare the LCD to receive the data
		lcd_startdata();
		
		// Trigger interrupt on VD rising (new frame)
		EXTI_ENABLE_VD
		
		/*
			Data is now being recorded by a chain of interrupts.
			VD will rise, enabling the HD interrupt
			HD will rise, clock in 256 bytes of data from DCLK, and reset the HD interrupt
			When 96 lines of data have been read in,
				interrupts finish and code execution returns here, with the
				image being sent over the serial port to the screen - not quite any more
			
		*/
		
		while( linenum < 96 ) {}
		
		// Finished sending the LCD data
		lcd_enddata();
		oled_erase_screen();
		
		u32 redpos	= sumredx / numred;
		redpos		= (96000*redpos) / 128000;
		if( redpos > 1 && redpos < 95 ) oled_rectangle( redpos, 20, redpos, 50, RED );
		
		if( servo_send_pulse == 0 ) {
			GPIO_WriteBit( GPIOB, GPIO_Pin_8, Bit_SET );
			GPIO_WriteBit( GPIOB, GPIO_Pin_9, Bit_SET );
			Delay( 0xFFFFFF );
			GPIO_WriteBit( GPIOB, GPIO_Pin_8, Bit_RESET );
			GPIO_WriteBit( GPIOB, GPIO_Pin_9, Bit_RESET );
			
			servo_send_pulse = 10;
		} else {
			servo_send_pulse--;
		}
		
	}

}

void flash_eyes( int n ) {
	//Turns the eyes/LEDs (GPIOE_Pin15) and speaker (GPIOA_Pin1) on for a bit, then off for a bit, n times.
	//Written in assembler as a learning experience before attempting to store camera data
	//Kept because it's awesome
	asm(
		"ldr	r0,=0x40011800 + 0x10	\r\n"	//PORTE's BSRR (bit set/reset register)
		"ldr	r1,=0x40010800 + 0x10	\r\n"	//PORTA's BSRR (bit set/reset register)
		"ldr	r2,=1<<15		\r\n"	//set PIN15
		"ldr	r3,=1<<31		\r\n"	//reset PIN15
		"ldr	r4,=1<<1		\r\n"	//set PIN1
		"ldr	r5,=1<<17		\r\n"	//reset PIN1

		"loop:				\r\n"	//loop back here to flash the eyes

		"str	r2,[r0]			\r\n"	//turn on PE15
		"str	r4,[r1]			\r\n"	//turn on PA1

		"ldr	r6,=0x222222		\r\n"	//store delay counter in r6
		"d1:				\r\n"	//busy wait loop
		"subs	r6,#1			\r\n"	//subtract 1 from r3, update status flags
		"bne	d1			\r\n"	//loop until r3 is 0

		"str	r3,[r0]			\r\n"	//turn off PE15
		"str	r5,[r1]			\r\n"	//turn off PA1

		"ldr	r6,=0x222222		\r\n"	//store delay counter in r6
		"d2:				\r\n"	//busy wait loop
		"subs	r6,#1			\r\n"	//subtract 1 from r3, update status flags
		"bne	d2			\r\n"	//loop until r3 is 0

		"subs	%0,#1			\r\n"	//subtract 1 from input n, update status flags
		"bne	loop			\r\n"	//flash the eyes again if we should

		:					//no output operands
		: "r"(n)				//one input, n, number of loops to perform
		: "cc", "r0", "r1", "r2", "r3", "r4", "r5", "r6"	//we clobber r0, r1, r2, r3 and the CPU flags change so let the compiler know
	);
}

void Delay( unsigned long delay ) {
	for(; delay; --delay );
}

void GPIO_Config() {

	// Configure PC10 as input pulled high
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	// Configure PC11 as input pulled high
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	// Configure PC12 as input pulled high
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	// Configure PE15 as push-pull output
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOE, &GPIO_InitStructure );

	// Configure PD0..7 as floating inputs
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	// Configure PE12, 13, 14 as floating inputs
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOE, &GPIO_InitStructure );

	// Connect PE12,13,14 (DCLK,HD,VD) to EXTI Line 12,13,14 for interrupt
	GPIO_EXTILineConfig( GPIO_PortSourceGPIOE, GPIO_PinSource12 );
	GPIO_EXTILineConfig( GPIO_PortSourceGPIOE, GPIO_PinSource13 );
	GPIO_EXTILineConfig( GPIO_PortSourceGPIOE, GPIO_PinSource14 );

	// Configure PA1 as push-pull output for the speaker
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Configure PA8 as push-pull output for TIM1 CH1
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Configure PA9 as open drain alt-function output (USART1 TX)
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_SetBits( GPIOA, GPIO_Pin_9 );

	// Configure PA10 as floating input (USART1 RX)
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Configure PB6 as open drain alt-function output (I2C1 SCL)
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	// Configure PB7 as open drain alt-function output (I2C1 SDA)
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	// Configure PA5, 6, 7 as push-pull alt function for SPI1
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Configure PB8, 9 as push-pull outputs for servos
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	// Configure PA2, 3 as push-pull outputs for LEDs on the LCD board and 4 for CS to the LCD/SD
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, Bit_SET );

}

void USART_Config() {
	// Configure USART1 at 28800 baud for the OLED
	USART_InitStructure.USART_BaudRate	= 28800;
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits	= USART_StopBits_1;
	USART_InitStructure.USART_Parity	= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode		= USART_Mode_Tx | USART_Mode_Rx;
	USART_Init( USART1, &USART_InitStructure );
	USART_Cmd( USART1, ENABLE );
}

void I2C_Config() {
	// Configure I2C1 for the camera
	I2C_InitStructure.I2C_Mode		= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle		= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1	= 0xAD;//0xAM --> ADAM
	I2C_InitStructure.I2C_Ack		= I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed	= 100000;
	I2C_Cmd( I2C1, ENABLE );
	I2C_Init( I2C1, &I2C_InitStructure );
}

void SPI_Config() {
	// Configure SPI1 for the LCD screen output
	SPI_InitStructure.SPI_Direction		= SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode		= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize		= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL		= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA		= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS		= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler	= SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit		= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial	= 7;
	SPI_Init( SPI1, &SPI_InitStructure );

	SPI_Cmd( SPI1, ENABLE );
}

void DMA_Config() {

	// Set up DMA1 Ch3 for sending data from memory to SPI1
	DMA_InitStructure.DMA_PeripheralBaseAddr=	(u32)0x4001300C;	//(SPI1)
	DMA_InitStructure.DMA_MemoryBaseAddr	=	(u32)data;
	DMA_InitStructure.DMA_DIR		=	DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize	=	256;
	DMA_InitStructure.DMA_PeripheralInc	=	DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc		=	DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize=	DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize	=	DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode		=	DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority		=	DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M		=	DMA_M2M_Disable;

	DMA_Init( DMA1_Channel3, &DMA_InitStructure );

}

void TIM_Config() {
	// Configure TIM1 timebase, TIM1CLK=72MHz, TIM1 Freq = clk/(period+1) = 72/12 = 6MHz
	TIM_TimeBaseStructure.TIM_Period	= 11;
	TIM_TimeBaseStructure.TIM_Prescaler	= 0;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter	= 0;
	TIM_TimeBaseInit( TIM1, &TIM_TimeBaseStructure );

	// Configure TIM1 CH1 for 50% duty cycle
	TIM_OCInitStructure.TIM_OCMode		= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState	= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 6;
	TIM_OCInitStructure.TIM_OCPolarity	= TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity	= TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState	= TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState	= TIM_OCNIdleState_Reset;
	TIM_OC1Init( TIM1, &TIM_OCInitStructure );

	//Enable the timer (starts generating pulses)
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void NVIC_Config() {
	// Enable the EXTI9_5 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel	= EXTI15_10_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	#ifdef  VECT_TAB_RAM
	  /* Set the Vector Table base location at 0x20000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif
}

void EXTI_Config() {

	// Enable an interrupt on EXTI line 13 rising (connected to HD)
	EXTI_InitStructure_HD.EXTI_Line		= EXTI_Line13;
	EXTI_InitStructure_HD.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTI_InitStructure_HD.EXTI_Trigger	= EXTI_Trigger_Rising;
	EXTI_InitStructure_HD.EXTI_LineCmd	= ENABLE;

	// Enable an interrupt on EXTI line 14 rising (connected to VD)
	EXTI_InitStructure_VD.EXTI_Line		= EXTI_Line14;
	EXTI_InitStructure_VD.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTI_InitStructure_VD.EXTI_Trigger	= EXTI_Trigger_Rising;
	EXTI_InitStructure_VD.EXTI_LineCmd	= ENABLE;
}

void EXTI15_10_IRQHandler() {
	if( EXTI_GetITStatus(EXTI_Line13) == SET ) {

		asm(
			//Load up register values needed to clock in the data
			"ldr	r0, =0x40011400 + 0x08	\r\n"	//PORTD's IDR (Input Data Register)
			"ldr	r1, =0x40011800 + 0x08	\r\n"	//PORTE's IDR (Input Data Register)
			"ldr	r2, =256		\r\n"	//Keep track of number of bytes stored, 256 bytes = 128 pixels
			"mov	r4, %0			\r\n"	//Copy the data address into r6 for later usage

			//Wait while the data clock is still high
			"dclk_high:			\r\n"	//Wait for DCLK to fall, indicating valid data
			"ldr	r3, [r1]		\r\n"	//Load PORTE_IDR into r3
			"tst	r3, #1<<12		\r\n"	//AND r3 with Pin12, setting flags, discarding results
			"bne	dclk_high		\r\n"	//If the AND operation was not 0, DCLK is still high, check again

			//Read in one byte of data
			"ldrb	r3, [r0]		\r\n"	//Load PORTD_IDR into r4, just the first byte
			"strb	r3, [%0]		\r\n"	//Store that byte into the frame array
			"add	%0, #1			\r\n"	//Increment pointer to the frame array to store the next byte
			"subs	r2, #1			\r\n"	//Decrement r2, which is counting the number of pixels to read
			"beq	end			\r\n"	//If r2 is now 0, we've read all we want, exit

			//Wait while the data clock is still low
			"dclk_low:			\r\n"	//Wait for DCLK to rise, so we can go back to waiting for it to fall
			"ldr	r3, [r1]		\r\n"	//Load PORTE_IDR into r3
			"tst	r3, #1<<12		\r\n"	//AND r3 with Pin12, setting flags, discarding results
			"beq	dclk_low		\r\n"	//If the AND operation was 0, DCLK is still low, check again
			"b	dclk_high		\r\n"	//Read in the next byte

			"end:				\r\n"	//Jump here once we've read in all the data we want

			//Configure and trigger the DMA request that will send the data to the LCD
			"ldr	r0, =0x40020000 + 0x30	\r\n"	//DMA1_CCR3 - configure DMA1 channel 3 for SPI1_TX
			"ldr	r1, =0x3090		\r\n"	//Settings for CCR3
			"str	r1, [r0]		\r\n"
			"ldr	r0, =0x40020000 + 0x34	\r\n"	//DMA1_CNDTR3 - number of data to transfer
			"ldr	r1, =256		\r\n"	//256 data to transfer
			"str	r1, [r0]		\r\n"
			"ldr	r0, =0x40020000 + 0x38	\r\n"	//DMA1_CPAR3 - Peripheral Address
			"ldr	r1, =0x4001300C		\r\n"	//Address of SPI1 Tx
			"str	r1, [r0]		\r\n"
			"ldr	r0, =0x40020000 + 0x3C	\r\n"	//DMA1_CMAR3 - Memory Address
			"str	r4, [r0]		\r\n"	//Data address
			"ldr	r0, =0x40020000 + 0x30	\r\n"	//DMA1_CCR3 - configure DMA1 channel 3 for SPI1_TX
			"ldr	r1, =0x3091		\r\n"	//Enable DMA1_CH3
			"str	r1, [r0]		\r\n"
			"ldr	r0, =0x40013000 + 0x04	\r\n"	//SPI1_CR2 - SPI1 control register 2, used to enable DMA Tx
			"ldr	r1, =0x02		\r\n"	//Enable DMA Tx request
			"str	r1, [r0]		\r\n"

			//Analyse the image to determine the position of red pixels
			"ldr	r0, =0			\r\n"	//Current pixel number
			"analyse_start:			\r\n"	//Return here
			"ldrb	r1, [r4]		\r\n"	//Load first byte into r1 from r4
			"lsr	r1, r1, #3		\r\n"	//Shift right three places so all that's left is red
			"cmp	r1, #22			\r\n"	//Compare red values to 22
			"bhs	analyse_further		\r\n"	//If red>=22 branch to analyse_further
			"add	r4, #2			\r\n"	//Point r4 to the next whole pixel (2 bytes on)
			"add	r0, #1			\r\n"	//Increment pixel number
			"cmp	r0, #128		\r\n"	//Compare pixel number to 128, the max
			"bne	analyse_start		\r\n"	//Return to analyse the next pixel if it's not at 128
			"b	disable_dma		\r\n"	//Otherwise go on to disable the DMA

			"analyse_further:		\r\n"	//If red>=22 go here
			"ldrb	r1, [r4]		\r\n"	//Load the first byte into r1
			"add	r4, #1			\r\n"	//Increment r4
			"ldrb	r2, [r4]		\r\n"	//Load the second byte into r2
			"add	r4, #1			\r\n"	//Increment r4 so it now points at the start of the next pixel
			"lsl	r1, #8			\r\n"	//Shift the first pixel up by eight
			"orr	r1, r1, r2		\r\n"	//Combine r1 and r2 so r1 now holds 16 bits, the entire pixel
			"and	r2, r1, #0x07E0		\r\n"	//Put greens in r2
			"and	r1, r1, #0x001F		\r\n"	//Put blues in r1
			"lsr	r2, #6			\r\n"	//Move greens into LSBs and chop off one bit to give same range as blues
			"cmp	r1, #13			\r\n"	//Compare blues to 13
			"it	ls			\r\n"	//If blues<=13
			"cmpls	r2, #13			\r\n"	//	then compare greens to 13
			"bls	analyse_update		\r\n"	//If greens<=13 branch to analyse_update, storing the x-val of the pixel
			"add	r0, #1			\r\n"	//Increment pixel number
			"cmp	r0, #128		\r\n"	//Compare to 128
			"bne	analyse_start		\r\n"	//Resume analysis if not finished
			"b	disable_dma		\r\n"	//Otherwise finished analysing

			"analyse_update:		\r\n"	//If the pixel is a red one we are interested in
			"ldr	r1, [%1]		\r\n"	//Load number of red pixels into r1
			"ldr	r2, [%2]		\r\n"	//Load sum of x-values into r2
			"add	r1, #1			\r\n"	//Add 1 to number of red pixels
			"add	r2, r0			\r\n"	//Add current x-value to running sum
			"str	r1, [%1]		\r\n"	//Store new number of red pixels
			"str	r2, [%2]		\r\n"	//Store new sum of x-values
			"add	r0, #1			\r\n"	//Increment pixel number
			"cmp	r0, #128		\r\n"	//Compare to 128
			"bne	analyse_start		\r\n"	//Resume analysis if not finished
								//Otherwise execution continues on to disabling DMA

			//Wait for the DMA request to end then disable the DMA channel
			"disable_dma:			\r\n"	//Label used to skip past image analysing part
			"ldr	r0, =0x40020000 	\r\n"	//DMA1_ISR
			"transfer_complete:		\r\n"
			"ldr	r1, [r0]		\r\n"	//Load DMA1_ISR into r1
			"tst	r1, #1<<9		\r\n"	//Test for TCIF3 (transfer complete ch3)
			"beq	transfer_complete	\r\n"	//Wait for it to be set to 1 by the DMA
			"ldr	r0, =0x40020000 + 0x04	\r\n"	//DMA1_IFCR
			"ldr	r1, =1<<9		\r\n"	//CTCIF3, clear transfer complete for ch3
			"str	r1, [r0]		\r\n"	//Clear the bit
			"ldr	r0, =0x40020000 + 0x30	\r\n"	//DMA1_CCR3
			"ldr	r1, =0x3090		\r\n"	//Disable DMA1_CH3
			"str	r1, [r0]		\r\n"
			"ldr	r0, =0x40013000 + 0x04	\r\n"	//SPI1_CR2
			"ldr	r1, =0x00		\r\n"	//Disable SP1 DMA Tx
			"str	r1, [r0]		\r\n"

			: 					//No output operands
			: "r"(data), "r"(numredptr), "r"(sumredxptr)//The start of the data array
			: "cc", "memory", "r0", "r1", "r2", "r3", "r4"//Tell the compiler we clobber the ALU flags, RAM and some registers
		);

		EXTI_DISABLE

		linenum++;
		if( linenum == 96 ) {
			return;
		} else {
			EXTI_ENABLE_HD
		}

	} else if( EXTI_GetITStatus(EXTI_Line14) == SET ) {
		EXTI_DISABLE
		EXTI_ENABLE_HD
	}
}

void Clock_Config() {
	
	// Reset RCC
	RCC_DeInit();
	
	// Enable HSE (High-Speed External oscillator)
	RCC_HSEConfig( RCC_HSE_ON );

	// Wait for the HSE to be ready
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if( HSEStartUpStatus == SUCCESS ) {
		// Enable Prefetch Buffer
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		// Flash 2 wait state
		FLASH_SetLatency(FLASH_Latency_2);

		// HCLK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		// PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);

		// PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);

		// PLLCLK = 8MHz * 9 = 72 MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		// Enable PLL
		RCC_PLLCmd(ENABLE);

		// Wait till PLL is ready
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		// Select PLL as system clock source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// Wait till PLL is used as system clock source
		while(RCC_GetSYSCLKSource() != 0x08) {}
	}

	// Initialise clock to peripherals

	RCC_AHBPeriphClockCmd(
		RCC_AHBPeriph_DMA1
		, ENABLE);

	RCC_APB1PeriphClockCmd(
		RCC_APB1Periph_I2C1
		, ENABLE );

	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_GPIOA |
		RCC_APB2Periph_GPIOB |
		RCC_APB2Periph_GPIOC |
		RCC_APB2Periph_GPIOD |
		RCC_APB2Periph_GPIOE |
		RCC_APB2Periph_AFIO  |
		RCC_APB2Periph_USART1|
		RCC_APB2Periph_TIM1  |
		RCC_APB2Periph_SPI1
		, ENABLE );

}
