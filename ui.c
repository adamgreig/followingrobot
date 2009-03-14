/************************************************
 *  FollowingRobot - ui.c
 *  Functions to provide a user interface
 *  Implementation file.
 *  See README for more details.
 ***********************************************/

#include "ui.h"

void ui_menu() {
    if( ui_menu_selection == UI_SELECT_TURNING ) {
        //Draw a rectangle below 'turning'
        oled_rectangle( 0, 0, 95, 14, RED );
    } else if( ui_menu_selection == UI_SELECT_DRIVING ) {
        //Draw a rectangle below 'driving'
        oled_rectangle( 0, 14, 95, 28, RED );
    } else if( ui_menu_selection == UI_SELECT_LIGHTS ) {
        //Draw a rectangle below 'lights'
        oled_rectangle( 0, 28, 95, 42, RED );
    }
    //Draw the text itself
    oled_formatted_string( 0, 0, 0, WHITE, "Turning On/Off" );
    Delay( 20000 );
    oled_formatted_string( 0, 2, 0, WHITE, "Driving On/Off" );
    Delay( 20000 );
    oled_formatted_string( 0, 4, 0, WHITE, "Lights On/Off" );
}

void ui_check() {
    //UP being pressed
    if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_10 ) == 0 ) {
        if( ui_menu_selection == UI_SELECT_TURNING ) ui_menu_selection = UI_SELECT_LIGHTS;
        else if( ui_menu_selection == UI_SELECT_DRIVING ) ui_menu_selection = UI_SELECT_TURNING;
        else ui_menu_selection = UI_SELECT_DRIVING;

    //DOWN being pressed
    } else if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_12 ) == 0 ) {
        if( ui_menu_selection == UI_SELECT_TURNING ) ui_menu_selection = UI_SELECT_DRIVING;
        else if( ui_menu_selection == UI_SELECT_DRIVING ) ui_menu_selection = UI_SELECT_LIGHTS;
        else ui_menu_selection = UI_SELECT_TURNING;

    //SELECT being pressed
    } else if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_11 ) == 0 ) {
        if( ui_menu_selection == UI_SELECT_TURNING ) {
            turning_enabled = !turning_enabled;
            flash_eyes(1);
        } else if( ui_menu_selection == UI_SELECT_DRIVING ) {
            driving_enabled = !driving_enabled;
            flash_eyes(1);
        } else if( ui_menu_selection == UI_SELECT_LIGHTS ) {
            lights_enabled = !lights_enabled;
            if( lights_enabled ) GPIO_SetBits( GPIOE, GPIO_Pin_15 );
            else GPIO_ResetBits( GPIOE, GPIO_Pin_15 );
        }
    }
}
