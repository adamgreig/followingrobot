/************************************************
 *  FollowingRobot - ui.h
 *  Functions to provide a user interface
 *  Header file.
 *  See README for more details.
 ***********************************************/

#include "oled.h"

//tracking_enabled is defined in main.c
extern u8 tracking_enabled;

//The messages that will be shown at some point
// in startup
#define UI_STARTUP_INITIALISING 1
#define UI_STARTUP_CONFIRM 2
#define UI_STARTUP_RESTART 3

//The different menu options that might be selected
#define UI_SELECT_TRACKING 1
#define UI_SELECT_SD 2

//Global state variables store the current selection,
// whether the button was pressed and whether something
// happened this frame (to prevent multiple actions)
int UI_MENU_SELECTION = 0;
int UI_MENU_ACTION = 0;
int UI_LOCK = 0;

//ui_startup: show one of the startup messages
void ui_startup(int message);

//ui_menu: display the menu, highlighting the selected choice
void ui_menu();

//ui_check: very quickly check inputs, setting appropriate
// selection/action and locking input for that frame
void ui_check();

//ui_process: respond to any action set in the frame processing
void ui_process();
