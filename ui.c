/************************************************
 *  FollowingRobot - ui.c
 *  Functions to provide a user interface
 *  Implementation file.
 *  See README for more details.
 ***********************************************/

#include "ui.h"

void ui_menu() {
    if( ui_menu_selection == UI_SELECT_TRACKING ) {
        //Draw a rectangle below 'tracking'
        oled_rectangle( 0, 0, 95, 12, RED );
    } else if( ui_menu_selection == UI_SELECT_SD ) {
        //Draw a rectangle below 'SD'
        oled_rectangle( 0, 12, 95, 24, RED );
    }
    //Draw the text itself
    //oled_formatted_string( 0, 0, 0, WHITE, "Tracking ON/OFF" );
    oled_formatted_string( 0, 2, 0, WHITE, "Toggle SD/LCD" );
}

void ui_check() {
    //UP or DOWN being pressed
    if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_10 ) == 0 || GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_12 ) == 0 ) {
        if( ui_menu_selection == UI_SELECT_TRACKING ) ui_menu_selection = UI_SELECT_SD;
        else ui_menu_selection = UI_SELECT_TRACKING;
    //SELECT being pressed
    } else if( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_11 ) == 0 ) {
        if( ui_menu_selection == UI_SELECT_TRACKING ) tracking_enabled = !tracking_enabled;
        //else if( ui_menu_selection == UI_SELECT_SD )
    }
}
