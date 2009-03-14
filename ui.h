/************************************************
 *  FollowingRobot - ui.h
 *  Functions to provide a user interface
 *  Header file.
 *  See README for more details.
 ***********************************************/

//OLED functions. Also includes the STM32 library
#include "oled.h"

//provided by main.c
void flash_eyes( int n );
void Delay( unsigned long delay );

//tracking_enabled is defined in main.c
extern volatile u8 turning_enabled;
extern volatile u8 driving_enabled;
extern volatile u8 lights_enabled;

//keep track of what menu option is selected
extern volatile u8 ui_menu_selection;

//The different menu options that might be selected
#define UI_SELECT_TURNING 1
#define UI_SELECT_DRIVING 2
#define UI_SELECT_LIGHTS 3

//ui_menu: display the menu, highlighting the selected choice
void ui_menu();

//ui_check: check inputs, reaction appropriately either changing
// menu selection or doing whatever was selected
void ui_check();
