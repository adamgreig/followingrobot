/************************************************
 *  FollowingRobot - ui.c
 *  Functions to provide a user interface
 *  Implementation file.
 *  See README for more details.
 ***********************************************/

#include "ui.h"

void ui_startup(int message) {
    if( message == UI_STARTUP_INITIALISING ) {
        oled_erase_screen();
        oled_formatted_string( 1, 0, 0, WHITE, "Initialising..." );
    } else if( message == UI_STARTUP_CONFIRM ) {
        oled_erase_screen();
        oled_formatted_string( 0, 0, 0, WHITE, "Does the screen" );
        oled_formatted_string( 0, 1, 0, WHITE, "show valid data?" );
        oled_formatted_string( 0, 2, 0, WHITE, "UP:   Yes" );
        oled_formatted_string( 0, 3, 0, WHITE, "DOWN: No" );
    }
}

void ui_menu() {
    if( UI_MENU_SELECTION == UI_SELECT_TRACKING ) {
        //Draw a rectangle below 'tracking'
        oled_rectangle( 0, 0, 96, 12, RED );
    } else if( UI_MENU_SELECTION == UI_SELECT_SD ) {
        //Draw a rectangle below 'SD'
        oled_rectangle( 0, 12, 96, 24, RED );
    }
    //Draw the text itself
    oled_formatted_string( 0, 0, 0, BLACK, "Tracking ON/OFF" );
    oled_formatted_string( 0, 1, 0, BLACK, "Toggle SD/LCD" );
}

void ui_check() {
    //Assembler used for speed
    asm(
        "ldr    0x00    0x00        \r\n"   //Placeholder
        :                                   //Outputs
        :                                   //Inputs
        :                                   //Clobber
    );
}

void ui_process() {
    //This keeps more than one action happening per frame
    if( UI_LOCK ) UI_LOCK = 0;

    //If the button was pressed then we have to do something about it
    if( UI_MENU_ACTION ) {
        if( UI_MENU_SELECTION == UI_SELECT_TRACKING ) {
            tracking_enabled = !tracking_enabled;
        }
    }
}
