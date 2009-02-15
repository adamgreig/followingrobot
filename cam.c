/************************************************
 *	FollowingRobot - cam.c
 *	Functions to initialise the small
 *	digital camera from SparkFun.
 *	Implementation file.
 *	See README for more details.
 ***********************************************/

 #include "cam.h"

 void cam_init() {
	 // Start the camera sending data
	 //Turn camera on, resolution 128x96, format RGB565, Colour
	I2C_GenerateSTART( I2C1, ENABLE );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress( I2C1, CAM_ADDR, I2C_Direction_Transmitter );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData( I2C1, 0x03 );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData( I2C1, 0x22 );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP( I2C1, ENABLE );

	// Configure a few more options
	//Turn off sync codes
	I2C_GenerateSTART( I2C1, ENABLE );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress( I2C1, CAM_ADDR, I2C_Direction_Transmitter );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData( I2C1, 0x1E );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData( I2C1, 0x48 );
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP( I2C1, ENABLE );
 }
 