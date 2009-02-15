/************************************************
 *	FollowingRobot - cam.h
 *	Functions to initialise the small
 *	digital camera from SparkFun.
 *	Header file.
 *	See README for more details.
 ***********************************************/

#include "stm32f10x_lib.h"
 
//Camera's I2C address
#define CAM_ADDR 0x78

//Initialise the camera, setting all required settings
void cam_init();