/************************************************
 *	FollowingRobot - servo.h
 *	Control servos on PB8,9
 *	Header file.
 *	See README for more details.
 ***********************************************/

#include "stm32f10x_lib.h"

//Values found by experimentation to reliably send that servo
// in that direction.
#define SERVO_L_BACKWARD 13800
#define SERVO_L_FORWARD	16800

#define SERVO_R_BACKWARD 10200
#define SERVO_R_FORWARD	7500

#define SERVO_L		1
#define SERVO_R		2

//Send a pulse to a specified servo, moving it
void servo_send_pulse( int servo, int pulse );

//Prototype for Delay, defined in main.c, a busy wait loop
void Delay( unsigned long delay );