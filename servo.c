/************************************************
 *	FollowingRobot - servo.c
 *	Control servos on PB8,9
 *	Implementation file.
 *	See README for more details.
 ***********************************************/

#include "servo.h"

void servo_send_pulse( int servo, int pulse ) {
	if( servo == SERVO_L ) {
		GPIO_WriteBit( GPIOB, GPIO_Pin_8, Bit_SET );
		Delay( pulse );
		GPIO_WriteBit( GPIOB, GPIO_Pin_8, Bit_RESET );
	} else if( servo == SERVO_R ) {
		GPIO_WriteBit( GPIOB, GPIO_Pin_9, Bit_SET );
		Delay( pulse );
		GPIO_WriteBit( GPIOB, GPIO_Pin_9, Bit_RESET );
	}
}
