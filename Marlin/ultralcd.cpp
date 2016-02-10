#include "temperature.h"
#include "ultralcd.h"

#ifdef DAC_DRIVER
#include "mcp4728.h"
#define DAC_MIN 0
#define DAC_MAX 5000
#define DAC_SCALAR 50
#endif

#ifdef ULTRA_LCD
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "ConfigurationStore.h"

bool lcdFastUpdate = false;                 //  Used for higher update scrolling of SD card filenames
bool lcdLongTimeout = false;                //  Used to lengthen timeout to status screen (for leveling & filament change, etc.)
uint32_t timeoutToStatus = 0;               //  Timeout tracking
bool invertEncoderDir = false;              //  Used to invert encoder direction for menu navigation when used with up/down buttons
bool encoderCoarseEnabled = false;          //  Used to invert encoder direction for menu navigation when used with up/down buttons
char strTemp[30];                           //  Used for building dynamic strings

/* Configuration settings */
int plaPreheatHotendTemp;
int plaPreheatHPBTemp;
int plaPreheatFanSpeed;

int absPreheatHotendTemp;
int absPreheatHPBTemp;
int absPreheatFanSpeed;
/* !Configuration settings */

//Function pointer to menu functions.
typedef void (*menuFunc_t)();

uint8_t lcd_status_message_level;
char lcd_status_message[LCD_WIDTH+1] = WELCOME_MSG;

#include "ultralcd_implementation_RigidBot.h"

/** forward declerations **/

void copy_and_scalePID_i();
void copy_and_scalePID_d();

/* Different menus */
static void lcd_status_screen();
static void lcd_return_to_status();
#ifdef ULTIPANEL
static void lcd_main_menu();
static void lcd_quick_menu();
static void lcd_tune_menu();
static void lcd_heat_cool_menu();
static void lcd_bed_level_menu();
static void lcd_move_menu();
static void lcd_jog_menu();
static void lcd_filament_menu();
static void lcd_control_menu();
//static void lcd_misc_settings_menu();
static void lcd_control_temperature_menu();
static void lcd_control_temperature_preheat_pla_settings_menu();
static void lcd_control_temperature_preheat_abs_settings_menu();
static void lcd_control_motion_menu();
static void lcd_control_retract_menu();
#ifdef DAC_DRIVER
static void init_dac();
static void lcd_driver_x();
static void lcd_driver_y();
static void lcd_driver_z();
static void lcd_driver_e();
static void save_dac();
static void lcd_dac_menu();
#endif
//static void lcd_utilities_menu();
static void lcd_sdcard_menu();

static void lcd_quick_feedback();//Cause an LCD refresh, and give the user visual or audiable feedback that something has happend

/* Different types of actions that can be used in menuitems. */
static void menu_action_back(menuFunc_t data);
static void menu_action_submenu(menuFunc_t data);
static void menu_action_gcode(const char* pgcode);
static void menu_action_function(menuFunc_t data);
static void menu_action_dynamicFunction(menuFunc_t data, const char* str);
static void menu_action_sdfile(const char* filename, char* longFilename, uint8_t item);
static void menu_action_sddirectory(const char* filename, char* longFilename, uint8_t item);
static void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
static void menu_action_setting_edit_float3(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float3_signed(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float32(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float5(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float51(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_float52(const char* pstr, float* ptr, float minValue, float maxValue);
static void menu_action_setting_edit_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue);
static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float3(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float32(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float5(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float51(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_float52(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
static void menu_action_setting_edit_callback_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue, menuFunc_t callbackFunc);

//#define ENCODER_FEEDRATE_DEADZONE 10
#define ENCODER_FEEDRATE_DEADZONE 0
#define ENCODER_STEPS_PER_MENU_ITEM  1

/* Helper macros for menus */
#define START_MENU() do { \
    invertEncoderDir = true; \
    if (encoderPosition > 0x8000 || encoderPosition < 0 ) encoderPosition = 0; \
    if (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM < currentMenuViewOffset) currentMenuViewOffset = encoderPosition / ENCODER_STEPS_PER_MENU_ITEM;\
    uint8_t _lineNr = currentMenuViewOffset, _menuItemNr; \
    for(uint8_t _drawLineNr = 0; _drawLineNr < LCD_HEIGHT; _drawLineNr++, _lineNr++) { \
        _menuItemNr = 0;
#define MENU_ITEM(type, label, args...) do { \
    if (_menuItemNr == _lineNr) { \
        if (lcdDrawUpdate) { \
            const char* _label_pstr = PSTR(label); \
            if ((encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) == _menuItemNr) { \
                lcd_implementation_drawmenu_ ## type ## _selected (_drawLineNr, _label_pstr , ## args ); \
            }else{\
                lcd_implementation_drawmenu_ ## type (_drawLineNr, _label_pstr , ## args ); \
            }\
        }\
        if ((LCD_CLICKED) && (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) == _menuItemNr) {\
            lcd_quick_feedback(); \
            menu_action_ ## type ( args ); \
            return;\
        }\
    }\
    _menuItemNr++;\
} while(0)
#define MENU_ITEM_BACK_HIDDEN(type, label, args...) do { \
    if ( buttons&B_LE ) { \
        lcd_quick_feedback(); \
        menu_action_ ## type ( args ); \
        return;\
    } \
} while(0)
#ifdef HIDE_BACK_MENUS
#define MENU_ITEM_BACK  MENU_ITEM_BACK_HIDDEN
#else
#define MENU_ITEM_BACK(type, label, args...) do { \
    MENU_ITEM_BACK_HIDDEN(type, label, ## args); \
    MENU_ITEM(type, label, ## args); \
} while(0)
#endif
#define MENU_ITEM_DUMMY() do { _menuItemNr++; } while(0)
#define MENU_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label) , ## args )
#define MENU_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label) , ## args )
#define END_MENU() \
    if (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM >= _menuItemNr) encoderPosition = _menuItemNr * ENCODER_STEPS_PER_MENU_ITEM - 1; \
    if ((uint8_t)(encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) >= currentMenuViewOffset + LCD_HEIGHT) { currentMenuViewOffset = (encoderPosition / ENCODER_STEPS_PER_MENU_ITEM) - LCD_HEIGHT + 1; lcdDrawUpdate = 1; _lineNr = currentMenuViewOffset - 1; _drawLineNr = -1; } \
    } } while(0)

/** Used variables to keep track of the menu */
volatile uint8_t buttons;//Contains the bits of the currently pressed buttons.
volatile uint8_t buttonHold;//Contains the bits of the currently pressed buttons.

uint8_t currentMenuViewOffset;              /* scroll offset in the current menu */
uint32_t blocking_enc;
uint8_t lastEncoderBits;
int8_t encoderDiff; /* encoderDiff is updated from interrupt context and added to encoderPosition every LCD update */
int8_t encoderDiff2;
int32_t encoderPosition; // this was originally uint32_t but I changed it.
#if (SDCARDDETECT > 0)
bool lcd_oldcardstatus;
#endif
#endif//ULTIPANEL

menuFunc_t currentMenu = lcd_status_screen; /* function pointer to the currently active menu */
menuFunc_t lastMenu = currentMenu;
uint32_t lcd_next_update_millis;
uint32_t lcd_status_update_millis;
//uint8_t lcd_status_update_delay;
uint8_t lcdDrawUpdate = 2;                  /* Set to none-zero when the LCD needs to draw, decreased after every draw. Set to 2 in LCD routines so the LCD gets atleast 1 full redraw (first redraw is partial) */

//prevMenu and prevEncoderPosition are used to the previous menu location when editing settings.
menuFunc_t prevMenu = NULL;
int16_t prevEncoderPosition; // this was originally uint16_t but I changed it.
//Variables used when editing values.
const char* editLabel;
void* editValue;
int32_t minEditValue, maxEditValue;
menuFunc_t callbackFunc;

#ifdef DAC_DRIVER
float channelVal;
int16_t firstCall = 0;
uint16_t driverX, driverY, driverZ, driverE;
//mcp4728 dac = mcp4728(1);
#endif

// placeholders for Ki and Kd edits
float raw_Ki, raw_Kd;


bool        lcdShowAbout;
uint32_t    lcdAboutTimeoutMillis;

/*
static void softwareReset(){
    // http://forum.arduino.cc/index.php/topic,42205.0.html
    WDTCR=0x18;
    WDTCR=0x08;
    asm("wdr");
    while(1);
}*/

static void lcd_show_about( uint32_t timeout )
{
    lcd_return_to_status();
    lcdShowAbout = true;
    lcdAboutTimeoutMillis = millis() + timeout;
    lcdDrawUpdate = 2;
}

static void lcd_show_about()
{
    lcd_show_about( 20000 );
}

static void lcd_show_about_cancel()
{
    lcdShowAbout = false;
    lcdAboutTimeoutMillis = 0;  
    lcdDrawUpdate = 2;
}

/* Main status screen. It's up to the implementation specific part to show what is needed. As this is very display dependent */
static void lcd_status_screen()
{
    static int feedLast = 0;    //  For tracking feedmultiply changes
    
    if ( millis() >= lcd_status_update_millis )
        lcdDrawUpdate = 1;

    if ( lcdShowAbout && millis() >= lcdAboutTimeoutMillis )
        lcd_show_about_cancel();

    if (lcdDrawUpdate)
    {   
        if ( lcdShowAbout )
            lcd_implementation_about_screen();
        else
            lcd_implementation_status_screen();
        lcd_status_update_millis = millis() + LCD_STATUS_UPDATE_INTERVAL;
    }

    // if right button clicked
    if (LCD_CLICKED || buttons&B_RI) //jkl;
    {
        encoderPosition = 0;
        lcd_quick_feedback();
        if ( lcdShowAbout )
            lcd_show_about_cancel();
        else if(buttons&B_RI) 
            currentMenu = lcd_quick_menu;
        else
            currentMenu = lcd_main_menu;
    }


    // Dead zone at 100% feedrate
    if (feedmultiply < 100 && (feedmultiply + int(encoderPosition)) > 100 ||
            feedmultiply > 100 && (feedmultiply + int(encoderPosition)) < 100)
    {
        encoderPosition = 0;
        feedmultiply = 100;
    }

    if (feedmultiply == 100 && int(encoderPosition) > ENCODER_FEEDRATE_DEADZONE)
    {
        feedmultiply += int(encoderPosition) - ENCODER_FEEDRATE_DEADZONE;
        encoderPosition = 0;
    }
    else if (feedmultiply == 100 && int(encoderPosition) < -ENCODER_FEEDRATE_DEADZONE)
    {
        feedmultiply += int(encoderPosition) + ENCODER_FEEDRATE_DEADZONE;
        encoderPosition = 0;    
    }
    else if (feedmultiply != 100)
    {
        feedmultiply += int(encoderPosition);
        encoderPosition = 0;
    }

    if (feedmultiply < 10)
        feedmultiply = 10;
    if (feedmultiply > 999)
        feedmultiply = 999;
        
    if ( feedmultiply != feedLast )
        lcd_status_update_millis = 0;   //  Force update on next cycle
        //lcd_status_update_delay = 0;  //  Force update on next cycle
    feedLast = feedmultiply;
}

#ifdef ULTIPANEL


static void lcd_draw_return_message(uint8_t row)
{
    lcd.setCursor(1, row);
    lcd_printPGM(PSTR("Press center to quit"));
}

//####################################################################################################
//  Main status display screen
//####################################################################################################
static float   oldX;
static float   oldY;
static float   oldZ;
static float   oldE;
static float   oldFeedrate;
static bool    using_sd_card = false;

static void lcd_return_to_status()
{
    encoderPosition = 0;
    currentMenu = lcd_status_screen;
}

void lcd_sdcard_pause() // Note: cannot add more commands than BUFSIZE-BUF_FILL_SIZE.
{
    oldX = current_position[X_AXIS];
    oldY = current_position[Y_AXIS];
    oldZ = current_position[Z_AXIS];
    oldE = current_position[E_AXIS];
    oldFeedrate = feedrate;

    // retract extruder (E axis)
    sprintf_P(strTemp, PSTR("G1 E%s F1800"), ftostr74(oldE-4.0));
    enquecommand(strTemp);

    //raise extruder (Z axis)
    sprintf_P(strTemp, PSTR("G0 Z%s F9000"), ftostr74(oldZ+5.0));
    enquecommand(strTemp);

    // move bed forward (Y axis) and extruder out of the way (X axis)
    sprintf_P(strTemp, PSTR("G0 X3.0 F9000") ); // G162
    enquecommand(strTemp);
    sprintf_P(strTemp, PSTR("G0 Y%s F5000"), ftostr74(Y_MAX_POS)); // G162 // add the add_homeing[1] offset here
    enquecommand(strTemp);

    
    if(card.sdprinting) {
        using_sd_card = true;
        //card.pauseSDPrint();
        sprintf_P(strTemp, PSTR("M25")); // pause SD card
        enquecommand(strTemp);
        lcdDrawUpdate = 2;
    }
    bedLeds.setLedSources(LED_BLINK1, LED_BLINK1, LED_OFF);
}
void lcd_sdcard_resume() // Note: cannot add more commands than BUFSIZE-BUF_FILL_SIZE.
{
    // move X and Y axis to zero quickly
    //sprintf_P(strTemp, PSTR("G0 F9000 Y1.0 X1.0"));
    //enquecommand(strTemp);
    
    // home Y, then X, just to make sure nothing was messed up
    sprintf_P(strTemp, PSTR("G28 X Y"));
    enquecommand(strTemp);
    
    // return X and Y to original position before the pause
    sprintf_P(strTemp, PSTR("G0 F5000 Y%s"), ftostr52(oldY));
    enquecommand(strTemp);
    sprintf_P(strTemp, PSTR("G0 F9000 X%s"), ftostr52(oldX));
    enquecommand(strTemp);
    

    // return Z to original position before the pause
    sprintf_P(strTemp, PSTR("G0 Z%s"), ftostr74(oldZ));
    enquecommand(strTemp);

    // un-retract extruder (E axis)
    sprintf_P(strTemp, PSTR("G1 E%s F1800"), ftostr74(oldE));
    enquecommand(strTemp);

    sprintf_P(strTemp, PSTR("G0 F%s"), ftostr6(oldFeedrate));
    enquecommand(strTemp);   


    // resume the print
    if(using_sd_card) {
        using_sd_card = false;
        //card.startFileprint();
        sprintf_P(strTemp, PSTR("M24 R")); // resume SD card
        enquecommand(strTemp);
        lcdDrawUpdate = 2;
    }
    bedLeds.setLedSources(LED_OFF, LED_ON, LED_ON);
}
static void lcd_sdcard_stop() // Note: cannot add more commands than BUFSIZE-BUF_FILL_SIZE.
{
    cancel_command = true;

    oldX = current_position[X_AXIS];
    oldY = current_position[Y_AXIS];
    oldZ = current_position[Z_AXIS];
    oldE = current_position[E_AXIS];
    
    // retract extruder (E axis)
    sprintf_P(strTemp, PSTR("G1 E%s F1800"), ftostr74(oldE-4.0));
    enquecommand(strTemp);

    //raise extruder (Z axis)
    sprintf_P(strTemp, PSTR("G0 Z%s"), ftostr74(oldZ+10.0));
    enquecommand(strTemp);

    // move bed forward (Y axis), move extruder out of way (X axis)
    sprintf_P(strTemp, PSTR("G0 F5000 Y%d X3.0"), Y_MAX_POS); // add the add_homeing[1] offset here
    enquecommand(strTemp);
    
    sprintf_P(strTemp, PSTR("M104 S0")); // turn off extruder
    enquecommand(strTemp);

    sprintf_P(strTemp, PSTR("M140 S0")); // turn off bed
    enquecommand(strTemp);

    sprintf_P(strTemp, PSTR("M107")); // turn off part fans
    enquecommand(strTemp);

    //card.pauseSDPrint();
    if(card.sdprinting){
        sprintf_P(strTemp, PSTR("M25")); // pause SD card
        enquecommand(strTemp);
    }
    
    bedLeds.setLedSources(LED_ON, LED_ON, LED_ON);

    //quickStop();
    if(SD_FINISHED_STEPPERRELEASE)
    {
        enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
    autotempShutdown();
    card.sdprinting = false;
    card.closefile();
    lcdDrawUpdate = 2;
}
static float prev_homeing[3] = {0,0,0};
static void lcd_move_origin_return(){
    current_position[X_AXIS] = current_position[X_AXIS] + add_homeing[0] - prev_homeing[0];
    current_position[Y_AXIS] = current_position[Y_AXIS] + add_homeing[1] - prev_homeing[1];
    current_position[Z_AXIS] = current_position[Z_AXIS] + add_homeing[2] - prev_homeing[2];
    
    prev_homeing[0] = add_homeing[0];
    prev_homeing[1] = add_homeing[1];
    prev_homeing[2] = add_homeing[2];
    
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    lcd_quick_feedback();
    menu_action_back(lcd_control_menu);
}

static void lcd_move_origin(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_move_origin_return); // runs the "lcd_move_origin_return" function and then returns to "lcd_control_menu"
    MENU_ITEM_EDIT(float32, "X Offset", &add_homeing[0], -X_MAX_POS, X_MAX_POS); //jkl;
    MENU_ITEM_EDIT(float32, "Y Offset", &add_homeing[1], -Y_MAX_POS, Y_MAX_POS);
    MENU_ITEM_EDIT(float32, "Z Offset", &add_homeing[2], -Z_MAX_POS, Z_MAX_POS);
    END_MENU();  
}

/* Menu implementation */
static void lcd_main_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_WATCH, lcd_status_screen);
#ifdef SDSUPPORT
    if (card.cardOK)
    {
        if (card.isFileOpen())
        {
            if (card.sdprinting)
                MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
            else
                MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
            MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
        }else{
            MENU_ITEM(submenu, MSG_CARD_MENU, lcd_sdcard_menu);
#if SDCARDDETECT < 1
            MENU_ITEM(gcode, MSG_CNG_SDCARD, PSTR("M21"));  // SD-card changed by user
#endif
        }
    }else{
        MENU_ITEM(submenu, MSG_NO_CARD, lcd_sdcard_menu);
#if SDCARDDETECT < 1
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
#endif
    }
#endif //SDSUPPORT
    /*
    if (movesplanned() || IS_SD_PRINTING)
    {
        MENU_ITEM(submenu, MSG_TUNE, lcd_tune_menu); //jkl;
    }*/
    
    MENU_ITEM(submenu, MSG_HEAT_COOL, lcd_heat_cool_menu);
    MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);
    MENU_ITEM(submenu, MSG_FILAMENTCHANGE, lcd_filament_menu);
    MENU_ITEM(submenu, MSG_BED_LEVEL, lcd_bed_level_menu);
    MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);
//    MENU_ITEM(submenu, MSG_UTILITIES, lcd_utilities_menu);
    END_MENU();
}

#ifdef SDSUPPORT
static void lcd_autostart_sd()
{
    card.lastnr=0;
    card.setroot();
    card.checkautostart(true);
}
#endif

//####################################################################################################
//  Tune / Prepare
//####################################################################################################

//  Tune will show while running
static void lcd_tune_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedmultiply, 10, 999);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
//#if TEMP_SENSOR_1 != 0
    //MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
//#endif
//#if TEMP_SENSOR_2 != 0
    //MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
//#endif
#if EXTRUDERS >= 2
MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if EXTRUDERS >= 3
MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extrudemultiply, 10, 999);
//#ifdef FILAMENTCHANGEENABLE
//     MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600"));
//#endif
    END_MENU();
}

//####################################################################################################
//  Motor Driver Gain control routines
//####################################################################################################
#ifdef DAC_DRIVER
/*
static void init_dac()
{
    MYSERIAL.print("DAC init");

    //dac.begin(); // just use voltages?
    //delay(10);
    //dac.setGain(0, 0, 0, 0);
    //dac.setVref(0, 0, 0, 0);
    //dac.vdd(DAC_MAX);
    
    driverX=0;
    driverY=0;
    driverZ=0;
    driverE=0;
    
    driverX = dac.getVout(0);
    driverY = dac.getVout(1);
    driverZ = dac.getVout(2);
    driverE = dac.getVout(3);
    
    


    if(driverX>0){
        driverX+=1;

    }
    if(driverY>0){
        driverY+=1;
    }
    if(driverZ>0){
        driverZ+=1;
    }
    if(driverE>0){
        driverE+=1;
    }
    
}
*/
static void lcd_driver_x()
{
    driverX = dac.getVout(0);
    if (encoderPosition != 0)
    {
        driverX += (int)encoderPosition * DAC_SCALAR;
        if (driverX < DAC_MIN)
            driverX = DAC_MIN;
        if (driverX > DAC_MAX)
            driverX = DAC_MAX;
        encoderPosition = 0;
        dac.voutWrite(driverX,driverY,driverZ,driverE);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("X Drive"), itostr4(driverX/50));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_dac_menu;
        encoderPosition = 0;
    }
}

static void lcd_driver_y()
{
    driverY = dac.getVout(1);
    if (encoderPosition != 0)
    {
        driverY += (int)encoderPosition *DAC_SCALAR;
        if (driverY < DAC_MIN)
        driverY = DAC_MIN;
        if (driverY > DAC_MAX)
        driverY = DAC_MAX;
        encoderPosition = 0;
        dac.voutWrite(driverX,driverY,driverZ,driverE);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Y Drive%"), itostr4(driverY/50));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_dac_menu;
        encoderPosition = 0;
    }
}

static void lcd_driver_z()
{
    driverZ = dac.getVout(2);
    if (encoderPosition != 0)
    {
        driverZ += (int)encoderPosition * DAC_SCALAR;
        if (driverZ < DAC_MIN)
        driverZ = DAC_MIN;
        if (driverZ > DAC_MAX)
        driverZ = DAC_MAX;
        encoderPosition = 0;
        dac.voutWrite(driverX,driverY,driverZ,driverE);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Z Drive%"), itostr4(driverZ/50));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_dac_menu;
        encoderPosition = 0;
    }
}

static void lcd_driver_e()
{
    driverE = dac.getVout(3);
    if (encoderPosition != 0)
    {
        driverE += (int)encoderPosition * DAC_SCALAR;
        if (driverE < DAC_MIN)
        driverE = DAC_MIN;
        if (driverE > DAC_MAX)
        driverE = DAC_MAX;
        encoderPosition = 0;
        dac.voutWrite(driverX,driverY,driverZ,driverE);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("E Drive%"), itostr4(driverE/50));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_dac_menu;
        encoderPosition = 0;
    }
}

static void save_dac()
{
    dac.eepromWrite();
}

static void lcd_dac_menu()
{
  // create and destroy the DAC object here
    //int a,b,c,d = 0; //These will be used to store the 4 dac channel values for printing and editing
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_control_menu);
    //MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, "Drive X", lcd_driver_x);
    MENU_ITEM(submenu, "Drive Y", lcd_driver_y);
    MENU_ITEM(submenu, "Drive Z", lcd_driver_z);
    MENU_ITEM(submenu, "Drive E", lcd_driver_e);
    //MENU_ITEM(function,"Save Values",save_dac);
    END_MENU();
}

#endif

//####################################################################################################
//  Temperature control routines
//####################################################################################################

int preheatType = 0;

static void lcd_preheat(int count, int type)
{
    int ext = type==0?plaPreheatHotendTemp:absPreheatHotendTemp;
    int bed = type==0?plaPreheatHPBTemp:absPreheatHPBTemp;
    setTargetHotend0(count>0?ext:0);
    setTargetHotend1(count>1?ext:0);
    setTargetHotend2(count>2?ext:0);
    setTargetBed(count>0?bed:0);
    fanSpeed = type==0?plaPreheatFanSpeed:absPreheatFanSpeed;
    lcd_return_to_status();
    setWatch(); // heater sanity check timer
}

static void lcd_cooldown()
{
    setTargetHotend0(0);
    setTargetHotend1(0);
    setTargetHotend2(0);
    setTargetBed(0);
    lcd_return_to_status();
}

static void lcd_preheat_all()
{
    lcd_preheat(3,preheatType);
}
static void lcd_preheat_one()
{
    lcd_preheat(1,preheatType);
}
static void lcd_preheat_two()
{
    lcd_preheat(2,preheatType);
}

static void lcd_preheat_select_menu()
{
    #if EXTRUDERS >= 2
    START_MENU();
    MENU_ITEM_BACK(back, MSG_HEAT_COOL, lcd_heat_cool_menu);
    MENU_ITEM(function, "All Extruders", lcd_preheat_all);
    MENU_ITEM(function, "Single Extruder", lcd_preheat_one);
    MENU_ITEM(function, "Dual Extruder", lcd_preheat_two);
    END_MENU();
    #else
    lcd_preheat_one();
    #endif
}

void lcd_preheat_pla()
{
    preheatType = 0;
    currentMenu = &lcd_preheat_select_menu;
    encoderPosition = 0;
}
void lcd_preheat_abs()
{
    preheatType = 1;
    currentMenu = &lcd_preheat_select_menu;
    encoderPosition = 0;
}

static void lcd_heat_cool_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(function, MSG_PREHEAT_PLA, lcd_preheat_pla);
    MENU_ITEM(function, MSG_PREHEAT_ABS, lcd_preheat_abs);
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
#if EXTRUDERS >= 2
    MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if EXTRUDERS >= 3
    MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_1_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
    END_MENU();
}


//static void lcd_utilities_menu()
//{
    //START_MENU();
    //MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    //MENU_ITEM(submenu, "Bed Leveling", lcd_bed_level_menu);
////#ifdef FILAMENTCHANGEENABLE
    ////MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600"));
////#endif
    //MENU_ITEM(gcode, MSG_FILAMENTCHANGE, );
    //END_MENU();
//}


//####################################################################################################
//  Manual movement routines
//####################################################################################################
float move_menu_scale;
static void lcd_move_menu_axis();

static void lcd_move_x()
{
    if (encoderPosition != 0)
    {
        current_position[X_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (current_position[X_AXIS] < X_MIN_POS+add_homeing[0])
            current_position[X_AXIS] = X_MIN_POS+add_homeing[0];
        if (current_position[X_AXIS] > X_MAX_POS+add_homeing[0])
            current_position[X_AXIS] = X_MAX_POS+add_homeing[0];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("X"), ftostr31(current_position[X_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_y()
{
    if (encoderPosition != 0)
    {
        current_position[Y_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (current_position[Y_AXIS] < Y_MIN_POS+add_homeing[1])
            current_position[Y_AXIS] = Y_MIN_POS+add_homeing[1];
        if (current_position[Y_AXIS] > Y_MAX_POS+add_homeing[1])
            current_position[Y_AXIS] = Y_MAX_POS+add_homeing[1];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 600, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Y"), ftostr31(current_position[Y_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_z()
{
    if (encoderPosition != 0)
    {
        current_position[Z_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (current_position[Z_AXIS] < Z_MIN_POS+add_homeing[2])
            current_position[Z_AXIS] = Z_MIN_POS+add_homeing[2];
        if (current_position[Z_AXIS] > Z_MAX_POS+add_homeing[2])
            current_position[Z_AXIS] = Z_MAX_POS+add_homeing[2];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 60, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Z"), ftostr31(current_position[Z_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_e()
{
    if (encoderPosition != 0)
    {
        current_position[E_AXIS] += float((int)encoderPosition) * move_menu_scale;
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 20, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Extruder"), ftostr31(current_position[E_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}

static void lcd_move_menu_axis()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MOVE_AXIS, lcd_move_menu);
    MENU_ITEM(submenu, "Move X", lcd_move_x);
    MENU_ITEM(submenu, "Move Y", lcd_move_y);
//    if (move_menu_scale < 10.0)
//    {
        MENU_ITEM(submenu, "Move Z", lcd_move_z);
        MENU_ITEM(submenu, "Extruder", lcd_move_e);
//    }
    END_MENU();
}

static void lcd_move_menu_10mm()
{
    move_menu_scale = 10.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_1mm()
{
    move_menu_scale = 1.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_01mm()
{
    move_menu_scale = 0.1;
    lcd_move_menu_axis();
}

static void lcd_move_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, "Jog Axis", lcd_jog_menu);
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    MENU_ITEM(submenu, "Move 10mm", lcd_move_menu_10mm);
    MENU_ITEM(submenu, "Move 1mm", lcd_move_menu_1mm);
    MENU_ITEM(submenu, "Move 0.1mm", lcd_move_menu_01mm);
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));
    //TODO:X,Y,Z,E
    END_MENU();
}

//####################################################################################################
//  Jog Mode Routines
//####################################################################################################
int jogMode = 0;
float jogIncrement = 0.0;
float jogSpeed = 0.0;

static void lcd_extruder_switch();  //  function prototype

static void lcd_jog()
{
    currentMenu = &lcd_jog;
    lcdFastUpdate = true;
    lcdLongTimeout = true;
    
    float   jogH = 0;
    float   jogV = 0;
    float   jogX = 0;
    float   jogY = 0;
    float   jogZ = 0;
    float   jogE = 0;

    if (buttonHold & B_RI)  jogH += jogIncrement;
    if (buttonHold & B_LE)  jogH -= jogIncrement;
    if (buttonHold & B_UP)  jogV += jogIncrement;
    if (buttonHold & B_DW)  jogV -= jogIncrement;

    switch( jogMode )
    {
        case 0: //  X/Y
            jogX = jogH;
            jogY = jogV;
        break;

        case 1: //  Z
            jogZ = jogV;
        break;

        case 2: //  E
            jogE = -jogV;
        break;
    }

    if ( (jogX != 0 || jogY != 0 || jogZ || jogE) && movesplanned() < 3 )
    {
        current_position[X_AXIS] += jogX;
        if (current_position[X_AXIS] < X_MIN_POS+add_homeing[0])
            current_position[X_AXIS] = X_MIN_POS+add_homeing[0];
        if (current_position[X_AXIS] > X_MAX_POS+add_homeing[0])
            current_position[X_AXIS] = X_MAX_POS+add_homeing[0];

        current_position[Y_AXIS] += jogY;
        if (current_position[Y_AXIS] < Y_MIN_POS+add_homeing[1])
            current_position[Y_AXIS] = Y_MIN_POS+add_homeing[1];
        if (current_position[Y_AXIS] > Y_MAX_POS+add_homeing[1])
            current_position[Y_AXIS] = Y_MAX_POS+add_homeing[1];

        current_position[Z_AXIS] += jogZ;
        if (current_position[Z_AXIS] < Z_MIN_POS+add_homeing[2])
            current_position[Z_AXIS] = Z_MIN_POS+add_homeing[2];
        if (current_position[Z_AXIS] > Z_MAX_POS+add_homeing[2])
            current_position[Z_AXIS] = Z_MAX_POS+add_homeing[2];

        current_position[E_AXIS] += jogE;

        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], jogSpeed, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        //lcd_implementation_drawedit(PSTR("X"), ftostr31(current_position[X_AXIS]));
        lcd.setCursor(0, 0);
        if ( jogMode == 0 )
        {
            lcd_printPGM(PSTR(" Jog X/Y Axis"));

            lcd.setCursor(1, 1);
            lcd_printPGM(PSTR("X: "));
            lcd.print(ftostr31(current_position[X_AXIS]));
            lcd_printPGM(PSTR("  "));

            lcd.setCursor(1, 2);
            lcd_printPGM(PSTR("Y: "));
            lcd.print(ftostr31(current_position[Y_AXIS]));
            lcd_printPGM(PSTR("  "));
        }
        else if ( jogMode == 1 )
        {
            lcd_printPGM(PSTR(" Jog Z Axis"));

            lcd.setCursor(1, 1);
            lcd_printPGM(PSTR("Z: "));
            lcd.print(ftostr31(current_position[Z_AXIS]));
            lcd_printPGM(PSTR("  "));
        }
        else if ( jogMode == 2 )
        {
            lcd_printPGM(PSTR(" Jog Extruder"));

            lcd.setCursor(1, 1);
            lcd_printPGM(PSTR("E"));
            lcd.print(active_extruder);
            lcd_printPGM(PSTR(": "));
            lcd.print(ftostr31(current_position[E_AXIS]));
            lcd_printPGM(PSTR("   "));
            
            #if EXTRUDERS >= 2
            lcd.setCursor(1, 2);
            lcd_printPGM(PSTR("Press > to switch"));
            #endif
        }
        lcd_draw_return_message(3);
    }

    if ( jogMode == 2 && buttons&B_RI )
    {
        lcd_quick_feedback();
        lcd_extruder_switch();
    }
        
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lastMenu;
        encoderPosition = 0;
    }
}

static void lcd_jog_xy()
{
    jogMode = 0;            //  X/Y
    jogIncrement = JOG_XY_INC;
    jogSpeed = JOG_XY_SPEED;
    lcd_jog();
}

static void lcd_jog_z()
{
    jogMode = 1;            //  Z
    jogIncrement = JOG_Z_INC;
    jogSpeed = JOG_Z_SPEED;
    lcd_jog();
}

static void lcd_jog_e()
{
    jogMode = 2;            //  E
    jogIncrement = JOG_E_INC;
    jogSpeed = JOG_E_SPEED;
    lcd_jog();
}

static void lcd_jog_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MOVE_AXIS, lcd_move_menu);
    MENU_ITEM(submenu, "Jog Z", lcd_jog_z);
    MENU_ITEM(submenu, "Jog X/Y", lcd_jog_xy);
    MENU_ITEM(submenu, "Jog Extruder", lcd_jog_e);
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    END_MENU();
}

//####################################################################################################
//  Filament change Routines
//####################################################################################################
int cleanState;


static void lcd_filament_run( int16_t increment )
{
    lcdFastUpdate = true;
    lcdLongTimeout = true;
    
    if ( movesplanned() < 3 )
    {
        current_position[E_AXIS] += increment;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], JOG_E_SPEED, active_extruder);
    }
    
    if (lcdDrawUpdate)
    {
        lcd.setCursor(1, 0);
        if ( increment >= 0 )
            lcd_printPGM(PSTR("Load Filament..."));
        else
            lcd_printPGM(PSTR("Unload Filament..."));
        
        lcd_draw_return_message(2);
    }
    
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lastMenu;
        encoderPosition = 0;
    }
}

static void lcd_filament_load()
{
    lcd_filament_run(JOG_E_INC);
}

static void lcd_filament_unload()
{
    lcd_filament_run(-JOG_E_INC);
}

static void lcd_filament_clean()
{
    static uint32_t cleanDelayMillis = 0;
    static uint16_t delay = 0;

    lcdFastUpdate = true;
    lcdLongTimeout = true;

    if ( !movesplanned() && millis() >= cleanDelayMillis )
    {
        if ( delay )
        {
            cleanDelayMillis = millis() + delay;
            delay = 0;
        }
        else
        {
            jogSpeed = JOG_E_SPEED;
            switch ( cleanState )
            {
                case    0:
                    //  Initialize stuff
    //              if ( current_position[Z_AXIS] < 20 ) current_position[Z_AXIS] = 20;
    //              jogSpeed = JOG_Z_SPEED;
                    current_position[E_AXIS] += 50;
                break;

                case    1:
                    current_position[E_AXIS] += 40;
                    delay = 100;
                break;
        
                case    2:
                    current_position[E_AXIS] -= 5;
                    delay = 2000;
                break;

            }
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], jogSpeed, active_extruder);
            if ( ++cleanState > 2 ) cleanState = 1;
        }
    }
    
    if (lcdDrawUpdate)
    {
        lcd.setCursor(1, 0);
        lcd_printPGM(PSTR("Cleaning Nozzle..."));
        lcd_draw_return_message(2);
    }
    
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lastMenu;
        encoderPosition = 0;
    }
}

static void lcd_extruder_switch()
{
    uint8_t next_extruder = active_extruder + 1;
    if ( next_extruder >= EXTRUDERS ) next_extruder = 0;
    sprintf_P(strTemp, PSTR("T%d"), next_extruder);
    enquecommand(strTemp);
}

static void lcd_filament_menu()
{
//  lcdLongTimeout = true;

    lastMenu = lcd_filament_menu;
    cleanState = 0;

    uint8_t next_extruder = active_extruder + 1;
    if ( next_extruder >= EXTRUDERS ) next_extruder = 0;
    sprintf_P(strTemp, PSTR("Next Extruder (%d" LCD_STR_ARROW_RIGHT "%d)"), active_extruder, next_extruder);

    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, "Load Filament", lcd_filament_load);
    MENU_ITEM(submenu, "Unload Filament", lcd_filament_unload);
    MENU_ITEM(submenu, "Clean Nozzle", lcd_filament_clean);
#if EXTRUDERS > 1
        MENU_ITEM(dynamicFunction, "", lcd_extruder_switch, strTemp);
#endif
    END_MENU();
}

//####################################################################################################
//  Bed leveling routines
//####################################################################################################
int levelStep = 0;
int levelStepLast = 0;
int levelType = 0;
bool levelHomed = false;
bool moveHead = false;

#define BED_LEVEL_Z_LIFT    5

//  Point list for bed leveling script
const uint16_t LEVEL_POINTS[][2] = {
    { BED_LEVEL_Z_LIFT, 4 },        //  First record is { Z_LIFT, NUM_POINTS }
    { X_MIN_POS+35, Y_CENTER_POS-50 }, //jkl;********
    { X_MIN_POS+35, Y_CENTER_POS+50 },
    { X_MAX_POS-35, Y_CENTER_POS+50 },
    { X_MAX_POS-35, Y_CENTER_POS-50 }
};

//  Point list for bed leveling extents script
const uint16_t LEVEL_EXTENTS[][2] = {
    { BED_LEVEL_Z_LIFT, 5 },        //  First record is { Z_LIFT, NUM_POINTS }
    { X_MIN_POS+10, Y_MIN_POS+10 },
    { X_MIN_POS+10, Y_MAX_POS-10 },
    { X_MAX_POS-10, Y_MAX_POS-10 },
    { X_MAX_POS-10, Y_MIN_POS+10 },
    { X_CENTER_POS, Y_CENTER_POS }
};

static void lcd_bed_level_run();    //  Function prototype

static void lcd_bed_level_doCurrent()
{
    //  Always move up a little first, and set high speed
    sprintf_P(strTemp, PSTR("G0 Z%d F10000"), LEVEL_POINTS[0][0]);
    enquecommand(strTemp);
    //  Home axes if not already homed
    if ( !levelHomed )
    {
        enquecommand_P(PSTR("G28"));        //  Home axes
        levelHomed = true;
        sprintf_P(strTemp, PSTR("G0 Z%d F10000"), LEVEL_POINTS[0][0]);
        enquecommand(strTemp);
    }
    if ( levelType == 0 )
    {
        if ( levelStep < LEVEL_POINTS[0][1] )
        {
            sprintf_P(strTemp, PSTR("G0 X%d Y%d"), LEVEL_POINTS[levelStep+1][0], LEVEL_POINTS[levelStep+1][1]);
            enquecommand(strTemp);
            enquecommand_P(PSTR("G0 Z0"));
        }
        else
        {
            currentMenu = &lcd_bed_level_menu;
        }
    }
    else
    {
        if ( levelStep < LEVEL_EXTENTS[0][1] )
        {
            sprintf_P(strTemp, PSTR("G0 X%d Y%d"), LEVEL_EXTENTS[levelStep+1][0], LEVEL_EXTENTS[levelStep+1][1]);
            enquecommand(strTemp);
            enquecommand_P(PSTR("G0 Z0"));
        }
        else
        {
            currentMenu = &lcd_bed_level_menu;
        }
    }
}
static void lcd_bed_level_doNext()
{
    levelStep++;
    lcd_bed_level_doCurrent();
}
static void lcd_bed_level_doLast()
{
    levelStep--;
    lcd_bed_level_doCurrent();
}
static void lcd_bed_level_moveHead()
{
    moveHead = true;
    sprintf_P(strTemp, PSTR("G0 Z%d F10000"), BED_LEVEL_Z_LIFT);
    enquecommand(strTemp);
    sprintf_P(strTemp, PSTR("G0 X%d Y%d"), X_CENTER_POS, Y_CENTER_POS);
    enquecommand(strTemp);
    encoderPosition = 0;
}
static void lcd_bed_level_moveHead_return()
{
    moveHead = false;
    lcd_bed_level_doCurrent();
}
static void lcd_bed_level_jog_return()
{
    sprintf_P(strTemp, PSTR("G0 Z%d F10000"), BED_LEVEL_Z_LIFT);
    enquecommand(strTemp);
    lcd_bed_level_doCurrent();
    currentMenu = lcd_bed_level_run;
    lcdDrawUpdate = 2;
}
static void lcd_bed_level_jog()
{
    lastMenu = lcd_bed_level_jog_return;
    lcd_jog_xy();
}
//static void lcd_bed_level_disable_steppers()
//{
//levelStep--;
//enquecommand_P(PSTR("M84"));
//levelHomed = false;
//encoderPosition = 0;
//}
static void lcd_bed_level_start()
{
    levelStep = -1;
    levelType = 0;
    lcd_bed_level_doNext();
}
static void lcd_bed_level_extents()
{
    levelStep = -1;
    levelType = 1;
    lcd_bed_level_doNext();
}
static void lcd_bed_level_cancel()
{
    lcd_bed_level_moveHead();
    lcd_bed_level_menu();
}
static void lcd_bed_level_syncZ()
{
    
}
static void lcd_bed_level_run()
{
    lastMenu = lcd_bed_level_run;

    if ( levelStep != levelStepLast )
    {
        lcd_quick_feedback();
        encoderPosition = 0;
    }
    levelStepLast = levelStep;

    START_MENU();
    if ( moveHead || (!levelHomed && levelStep >= 0) )
    {
        lcdLongTimeout = true;
        if ( moveHead )
            MENU_ITEM(function, "Resume", lcd_bed_level_moveHead_return);
        else
            MENU_ITEM(function, "Resume", lcd_bed_level_doNext);
        MENU_ITEM_BACK(back, "Cancel", lcd_bed_level_menu);
    }
    else if ( levelStep == -1 )
    {
        MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
        MENU_ITEM(function, "Level Bed", lcd_bed_level_start);
        MENU_ITEM(function, "Check Extents", lcd_bed_level_extents);
        //      MENU_ITEM(submenu, "Sync Z Steppers", lcd_bed_level_syncZ);
    }
    else
    {
        lcdLongTimeout = true;
        MENU_ITEM(function, "Continue", lcd_bed_level_doNext);
        MENU_ITEM(function, "Move Head", lcd_bed_level_moveHead);
        if ( levelStep > 0 )
        {
            MENU_ITEM(function, "Go Back", lcd_bed_level_doLast);
        }
        //MENU_ITEM(function, MSG_DISABLE_STEPPERS, lcd_bed_level_disable_steppers);
//      MENU_ITEM(function, "Manual Jog", lcd_bed_level_jog);
        MENU_ITEM(function, "Manual Jog", lcd_bed_level_jog);
        MENU_ITEM_BACK(back, "Cancel", lcd_bed_level_cancel);
    }
    END_MENU();
}

static void lcd_bed_level_menu()
{
    levelStep = -1;
    levelStepLast = 0;
    levelType = 0;
    levelHomed = false;
    moveHead = false;
    currentMenu = &lcd_bed_level_run;
}

//####################################################################################################
//  Control Menu Routines
//####################################################################################################
static void eeprom_RetrieveSettings(){
    Config_RetrieveSettings();
    dac.voutWrite(driverX,driverY,driverZ,driverE);
}

static void lcd_control_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
    MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM(submenu, "Move Origin", lcd_move_origin);
#ifdef FWRETRACT
    MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
#endif
#ifdef  DAC_DRIVER
        MENU_ITEM(submenu, MSG_DAC, lcd_dac_menu);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
    MENU_ITEM(function, MSG_LOAD_EPROM, eeprom_RetrieveSettings);
#endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, Config_ResetDefault);
    MENU_ITEM(function, "About", lcd_show_about);
    END_MENU();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
ToDo: 
Add misc settings menu
-make timing more accurate
*/

/*
 bool display_layer_num = false;
 bool autohome_between_layers = false;
 float every_other_layer = 0.8;
 bool disable_hbp_at_height = false;
 float height_var = 2.0;
 int new_bed_temp = 0;
 bool detect_end_of_print = false;
*/
static void lcd_do_nothing(){
    return;
}

/*
static void lcd_display_layer_num(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_misc_settings_menu);
    MENU_ITEM_EDIT(bool, "Display layer", &display_layer_num);
    MENU_ITEM(function, "-------------------", lcd_do_nothing);
    MENU_ITEM(function, "Display layer num", lcd_do_nothing);
    MENU_ITEM(function, "while printing.", lcd_do_nothing);
    END_MENU();
}

static void lcd_autohome_between_layers(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_misc_settings_menu);
    MENU_ITEM_EDIT(bool, "Home at height", &autohome_between_layers);
    MENU_ITEM_EDIT(float32,"Height (mm)",&every_other_layer,0,10000);
    MENU_ITEM(function, "-------------------", lcd_do_nothing);
    MENU_ITEM(function, "Home X and Y when", lcd_do_nothing);
    MENU_ITEM(function, "changing layers.", lcd_do_nothing);
    END_MENU();
}

static void lcd_disable_hbp_at_height(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_misc_settings_menu);
    MENU_ITEM_EDIT(bool, "Disable HB", &disable_hbp_at_height);
    MENU_ITEM_EDIT(float32,"at height",&height_var,1,50); // add this variable in
    MENU_ITEM_EDIT(int3, "New temp", &new_bed_temp, 0, 250);
    MENU_ITEM(function, "-------------------", lcd_do_nothing);
    MENU_ITEM(function, "Change heated bed", lcd_do_nothing);
    MENU_ITEM(function, "temp at height.", lcd_do_nothing);
    END_MENU();
}

static void lcd_detect_end_of_print(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_misc_settings_menu);
    MENU_ITEM_EDIT(bool, "Detect EOP", &detect_end_of_print);
    MENU_ITEM(function, "-------------------", lcd_do_nothing);
    MENU_ITEM(function, "Move bed forward", lcd_do_nothing);
    MENU_ITEM(function, "at end of print.", lcd_do_nothing);
    
    END_MENU();
}

static void lcd_misc_settings_menu(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_control_menu);
    //MENU_ITEM(submenu, "Display layer", lcd_display_layer_num);
    MENU_ITEM(submenu, "Home layers", lcd_autohome_between_layers);
    MENU_ITEM(submenu, "HB off at height", lcd_disable_hbp_at_height);
    //MENU_ITEM(submenu, "End of print", lcd_detect_end_of_print);
    END_MENU();

}
*/
static void lcd_control_temperature_menu()
{
    // set up temp variables - undo the default scaling
    raw_Ki = unscalePID_i(Ki);
    raw_Kd = unscalePID_d(Kd);

    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM(submenu, MSG_PREHEAT_PLA_SETTINGS, lcd_control_temperature_preheat_pla_settings_menu);
    MENU_ITEM(submenu, MSG_PREHEAT_ABS_SETTINGS, lcd_control_temperature_preheat_abs_settings_menu);
#ifdef AUTOTEMP
    MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &autotemp_enabled);
    MENU_ITEM_EDIT(float3, MSG_MIN, &autotemp_min, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float3, MSG_MAX, &autotemp_max, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float32, MSG_FACTOR, &autotemp_factor, 0.0, 1.0);
#endif
#ifdef PIDTEMP
    MENU_ITEM_EDIT(float52, MSG_PID_P, &Kp, 1, 9990);
    // i is typically a small value so allows values below 1
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I, &raw_Ki, 0.01, 9990, copy_and_scalePID_i);
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D, &raw_Kd, 1, 9990, copy_and_scalePID_d);
# ifdef PID_ADD_EXTRUSION_RATE
    MENU_ITEM_EDIT(float3, MSG_PID_C, &Kc, 1, 9990);
# endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
    END_MENU();
}

static void lcd_control_temperature_preheat_pla_settings_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &plaPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &plaPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &plaPreheatFanSpeed, 0, 255);
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}

static void lcd_control_temperature_preheat_abs_settings_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &absPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &absPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &absPreheatFanSpeed, 0, 255);
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}

static void lcd_control_motion_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(float5, MSG_ACC, &acceleration, 500, 99000);
    MENU_ITEM_EDIT(float3, MSG_VXY_JERK, &max_xy_jerk, 1, 990);
    MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &max_z_jerk, 0.1, 990);
    MENU_ITEM_EDIT(float3, MSG_VE_JERK, &max_e_jerk, 1, 990);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &max_feedrate[X_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &max_feedrate[Y_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &max_feedrate[Z_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &max_feedrate[E_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMIN, &minimumfeedrate, 0, 999);
    MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mintravelfeedrate, 0, 999);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &max_acceleration_units_per_sq_second[X_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &max_acceleration_units_per_sq_second[Y_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &max_acceleration_units_per_sq_second[Z_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &max_acceleration_units_per_sq_second[E_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT(float5, MSG_A_RETRACT, &retract_acceleration, 100, 99000);
    MENU_ITEM_EDIT(float52, MSG_XSTEPS, &axis_steps_per_unit[X_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float52, MSG_YSTEPS, &axis_steps_per_unit[Y_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ZSTEPS, &axis_steps_per_unit[Z_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ESTEPS, &axis_steps_per_unit[E_AXIS], 5, 9999);    
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    MENU_ITEM_EDIT(bool, "Endstop abort", &abort_on_endstop_hit);
#endif
    END_MENU();
}

#ifdef FWRETRACT
static void lcd_control_retract_menu()
{
    START_MENU();
    MENU_ITEM_BACK(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &autoretract_enabled);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate, 1, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate, 1, 999);
    END_MENU();
}
#endif

#if SDCARDDETECT == -1
static void lcd_sd_refresh()
{
    card.initsd();
    currentMenuViewOffset = 0;
}
#endif
static void lcd_sd_updir()
{
    card.updir();
    currentMenuViewOffset = 0;
}

void lcd_sdcard_menu()
{
    lcdFastUpdate = true;
    
    uint16_t fileCnt = card.getnrfilenames();
    START_MENU();
    MENU_ITEM_BACK(back, MSG_MAIN, lcd_main_menu);
    card.getWorkDirName();
    if(card.filename[0]=='/')
    {
#if SDCARDDETECT == -1
        MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
#endif
    }else{
        MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
    }
    
#ifdef LCD_REVERSE_FILE_ORDER
    for(int16_t i=fileCnt-1;i>=0;i--)
#else
    for(uint16_t i=0;i<fileCnt;i++)
#endif
    {
        if (_menuItemNr == _lineNr)
        {
            card.getfilename(i);
            if (card.filenameIsDir)
            {
                MENU_ITEM(sddirectory, MSG_CARD_MENU, card.filename, card.longFilename, _lineNr);
            }else{
                MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename, _lineNr);
            }
        }else{
            MENU_ITEM_DUMMY();
        }
    }
    END_MENU();
}


static void lcd_quick_menu(){
    START_MENU();
    MENU_ITEM_BACK(back, MSG_WATCH, lcd_status_screen);
    MENU_ITEM(submenu, "Jog Z", lcd_jog_z);
    MENU_ITEM(gcode, "Home X/Y", PSTR("G28 X Y"));
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
    #if EXTRUDERS >= 2
        MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
    #endif
    #if EXTRUDERS >= 3
        MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_1_MAXTEMP - 15);
    #endif
    #if TEMP_SENSOR_BED != 0
        MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
    #endif
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedmultiply, 10, 999);
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extrudemultiply, 10, 999);
    
    END_MENU();
}

//####################################################################################################
//  Menu edit routines
//####################################################################################################
#define menu_edit_type(_type, _name, _strFunc, scale) \
    void menu_edit_ ## _name () \
    { \
        encoderCoarseEnabled = true; \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
        } \
    } \
    void menu_edit_callback_ ## _name () \
    { \
        encoderCoarseEnabled = true; \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
            (*callbackFunc)();\
        } \
    } \
    static void menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) \
    { \
        encoderCoarseEnabled = true; \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
    }\
    static void menu_action_setting_edit_callback_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue, menuFunc_t callback) \
    { \
        encoderCoarseEnabled = true; \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_callback_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
        callbackFunc = callback;\
    }
menu_edit_type(int, int3, itostr3, 1)
menu_edit_type(float, float3, ftostr3, 1)
menu_edit_type(float, float3_signed, ftostr3_signed, 1)
menu_edit_type(float, float32, ftostr32, 100)
menu_edit_type(float, float5, ftostr5, 0.01)
menu_edit_type(float, float51, ftostr51, 10)
menu_edit_type(float, float52, ftostr52, 100)
menu_edit_type(unsigned long, long5, ftostr5, 0.01)


/** End of menus **/

static void lcd_quick_feedback()
{
    lcdDrawUpdate = 2;
    blocking_enc = millis() + 500;
    lcd_implementation_quick_feedback();
}

/** Menu action functions **/
static void menu_action_back(menuFunc_t data)
{
    currentMenu = data;
    encoderPosition = 0;
}
static void menu_action_submenu(menuFunc_t data)
{
    currentMenu = data;
    encoderPosition = 0;
}
static void menu_action_gcode(const char* pgcode)
{
    enquecommand_P(pgcode);
}
static void menu_action_function(menuFunc_t data)
{
    (*data)();
}
static void menu_action_dynamicFunction(menuFunc_t data, const char* str)
{
    (*data)();
}
static void menu_action_sdfile(const char* filename, char* longFilename, uint8_t item)
{
    char cmd[30];
    char* c;
    sprintf_P(cmd, PSTR("M23 %s"), filename);
    for(c = &cmd[4]; *c; c++)
        *c = tolower(*c);
    enquecommand(cmd);
    enquecommand_P(PSTR("M24"));
    lcd_return_to_status();
}
static void menu_action_sddirectory(const char* filename, char* longFilename, uint8_t item)
{
    card.chdir(filename);
    encoderPosition = 0;
}
static void menu_action_setting_edit_bool(const char* pstr, bool* ptr)
{
    *ptr = !(*ptr);
}
#endif//ULTIPANEL

/** LCD API **/
void lcd_init()
{
    lcd_implementation_init();
  #if BTN_ENC > 0
    pinMode(BTN_ENC,INPUT); 
    WRITE(BTN_ENC,HIGH);
  #endif    
  #ifdef RIGIDBOT_PANEL
    pinMode(BTN_UP,INPUT);
    pinMode(BTN_DWN,INPUT);
    pinMode(BTN_LFT,INPUT);
    pinMode(BTN_RT,INPUT);
 //   pinMode(BTN_ENT,INPUT)
  #endif

#if (SDCARDDETECT > 0)
    pinMode(SDCARDDETECT,INPUT);
    WRITE(SDCARDDETECT, HIGH);
    lcd_oldcardstatus = IS_SD_INSERTED;
#endif//(SDCARDDETECT > 0)
    lcd_buttons_update();
#ifdef ULTIPANEL    
    encoderDiff = 0;
#endif
}

void lcd_status_timeout_update()
{
    if ( lcdLongTimeout==true )
        timeoutToStatus = millis() + LCD_TIMEOUT_TO_STATUS_LONG;
    else
        timeoutToStatus = millis() + LCD_TIMEOUT_TO_STATUS;
}

void lcd_update()
{
//    static uint32_t timeoutToStatus = 0;
    static bool lastLongTimeout = false;

    static bool firstRun = true;
    if ( firstRun )
    {
        firstRun = false;
        lcd_show_about( 5000 );
    }
    
    lcd_buttons_update();
    
    #ifdef LCD_HAS_SLOW_BUTTONS
    buttons |= lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
    #endif
    
    #if (SDCARDDETECT > 0)
    if((IS_SD_INSERTED != lcd_oldcardstatus))
    {
        lcdDrawUpdate = 2;
        lcd_oldcardstatus = IS_SD_INSERTED;
        lcd_implementation_init(); // to maybe revive the lcd if static electricity killed it.
        
        if(lcd_oldcardstatus)
        {
            card.initsd();
            LCD_MESSAGEPGM(MSG_SD_INSERTED);
        }
        else
        {
            card.release();
            LCD_MESSAGEPGM(MSG_SD_REMOVED);
        }
    }
    #endif//CARDINSERTED
    
    if (lcd_next_update_millis < millis())
    {
       if (encoderDiff || encoderDiff2)
        {
            encoderDiff = invertEncoderDir?-encoderDiff:encoderDiff;    //  Check for direction inversion if set by a menu
            if ( encoderCoarseEnabled )                                 //  Check if left/right encoder is being used
            {
                encoderDiff2 *= ENCODER_COARSE_STEP;
                encoderDiff += encoderDiff2;
            }
            
            lcdDrawUpdate = 1;
            encoderPosition += encoderDiff; //  Get difference, 
            encoderDiff = 0;
            encoderDiff2 = 0;
            
            lcd_status_timeout_update();    
        }
        if (LCD_CLICKED || (lcdLongTimeout != lastLongTimeout)) //  Check for button press, or change in timeout status
            lcd_status_timeout_update();

        lastLongTimeout = lcdLongTimeout;   //  Store old value
        lcdLongTimeout = false;             //  Clear long timeout flag to allow a menu to set it
        encoderCoarseEnabled = false;       //  Clear encoder coarse enable to allow a menu to set it
        invertEncoderDir = false;           //  Clear encoder inversion to allow a menu to set it
        (*currentMenu)();

#ifdef LCD_HAS_STATUS_INDICATORS
        lcd_implementation_update_indicators();
#endif

//        if(timeoutToStatus < millis() && currentMenu != lcd_status_screen && !timeoutSuspend)
        if(timeoutToStatus < millis() && currentMenu != lcd_status_screen)
        {
            lcd_return_to_status();
            lcdDrawUpdate = 2;
        }
 
        if (lcdDrawUpdate == 2)
            lcd_implementation_clear();
        if (lcdDrawUpdate)
            lcdDrawUpdate--;

        if ( lcdFastUpdate )        //  Set up for update on next cycle
        {
            lcd_next_update_millis = millis() + LCD_FAST_UPDATE_INTERVAL;
            lcdDrawUpdate = 1;
        }
        else
            lcd_next_update_millis = millis() + LCD_UPDATE_INTERVAL;
        
    }
    lcdFastUpdate = false;
}

void lcd_setstatus(const char* message)
{
    if (lcd_status_message_level > 0)
        return;
    strncpy(lcd_status_message, message, LCD_WIDTH);
    lcdDrawUpdate = 2;
}
void lcd_setstatuspgm(const char* message)
{
    if (lcd_status_message_level > 0)
        return;
    strncpy_P(lcd_status_message, message, LCD_WIDTH);
    lcdDrawUpdate = 2;
}
void lcd_setalertstatuspgm(const char* message)
{
    lcd_setstatuspgm(message);
    lcd_status_message_level = 1;
#ifdef ULTIPANEL
    lcd_return_to_status();
#endif//ULTIPANEL
}
void lcd_reset_alert_level()
{
    lcd_status_message_level = 0;
}

#ifdef ULTIPANEL
/* Warning: This function is called from interrupt context */
void lcd_buttons_update()
{
    uint8_t newbuttons=0;
    static uint32_t repeatDelayMillis = 0;

    if(READ(BTN_ENC)==0){
        newbuttons |= EN_C;
    }
    if(READ(BTN_UP)==0)
        newbuttons |= B_UP;
    if(READ(BTN_DWN)==0)
        newbuttons |= B_DW;
    if(READ(BTN_RT)==0)
        newbuttons |= B_RI;
    if(READ(BTN_LFT)==0)
        newbuttons |= B_LE;

    buttonHold = newbuttons;            //  Store button hold states
    
    if ( millis() < blocking_enc )      //  Check if we are ignoring buttons
        newbuttons = 0;                 //  Clear out new button states
        
    if ( newbuttons != buttons || millis() > repeatDelayMillis )
    {
        //  Up/down buttons form main encoder
        if(newbuttons&B_UP)
            encoderDiff++;
        if(newbuttons&B_DW)
            encoderDiff--;

        //  Left/right buttons form secondary encoder (if needed)
        if(newbuttons&B_RI)
            encoderDiff2++;
        if(newbuttons&B_LE)
            encoderDiff2--;
            
        repeatDelayMillis = millis() + BUTTON_REPEAT_DELAY;
    }

    buttons = newbuttons;
}

void lcd_buzz(long duration, uint16_t freq)
{ 
#ifdef LCD_USE_I2C_BUZZER
  lcd.buzz(duration,freq);
#endif   
}

bool lcd_clicked() 
{ 
  return LCD_CLICKED;
}
#endif//ULTIPANEL

/********************************/
/** Float conversion utilities **/
/********************************/
//  convert float to string with +123.4 format
char conv[10];
char *ftostr3(const float &x)
{
  return itostr3((int)x);
}

char *itostr2(const uint8_t &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/10)%10+'0';
  conv[1]=(xx)%10+'0';
  conv[2]=0;
  return conv;
}

//  convert float to string with +/-123 format
char *ftostr3_signed(const float &x) //jkl;
{
  signed int xx=x;
  char sign = (xx>=0)?' ':'-'; //jkl;*****
  xx=abs(xx);
  conv[4] = 0;
  conv[3] = (xx)%10+'0';
  if(xx>= 10){
    conv[2] = (xx/10)%10+'0';
    if(xx >= 100){
        conv[1] = (xx/100)%10+'0';
        conv[0] = sign;
    }
    else{
        conv[1] = sign;
        conv[0] = ' ';
    }
  }
  else{
    conv[2] = sign;
    conv[1] = ' ';
    conv[0] = ' ';
  }
  return conv;
}

//  convert float to string with +123.4 format
char *ftostr31(const float &x)
{
  int xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

//  convert float to string with 123.4 format
char *ftostr31ns(const float &x)
{
  int xx=x*10;
  //conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[0]=(xx/1000)%10+'0';
  conv[1]=(xx/100)%10+'0';
  conv[2]=(xx/10)%10+'0';
  conv[3]='.';
  conv[4]=(xx)%10+'0';
  conv[5]=0;
  return conv;
}

char *ftostr32(const float &x)
{
  long xx=x*100;
  uint8_t pos = 0;
  if(xx < 0) conv[pos++]='-';
  else conv[pos++] = ' ';
  xx=abs(xx);
  conv[pos++]=(xx/10000)%10+'0';
  conv[pos++]=(xx/1000)%10+'0';
  conv[pos++]=(xx/100)%10+'0';
  conv[pos++]='.';
  conv[pos++]=(xx/10)%10+'0';
  conv[pos++]=(xx)%10+'0';
  conv[pos++]=0;
  return conv;
}

char *itostr31(const int &xx)
{
  conv[0]=(xx>=0)?'+':'-';
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr3(const int &x) //jkl; 
{
  int xx = x;
  char sign = (xx>=0)?' ':'-';
  xx = (xx>= 0)?xx:(~xx+1);
  conv[3]=0;
  conv[2]=(xx)%10+'0';
  if(xx >= 10){
    conv[1] = (xx/10)%10+'0';
    if(xx >= 100){
        conv[0] = (xx/100)%10 +'0';
    }
    else{
        conv[0] = sign;
    }
  }
  else{
    conv[1] = sign;
    conv[0] = ' ';
  }
  return conv;
}

char *itostr3left(const int &xx)
{
  if (xx >= 100)
  {
    conv[0]=(xx/100)%10+'0';
    conv[1]=(xx/10)%10+'0';
    conv[2]=(xx)%10+'0';
    conv[3]=0;
  }
  else if (xx >= 10)
  {
    conv[0]=(xx/10)%10+'0';
    conv[1]=(xx)%10+'0';
    conv[2]=0;
  }
  else
  {
    conv[0]=(xx)%10+'0';
    conv[1]=0;
  }
  return conv;
}

char *itostr4(const int &xx)
{
  if (xx >= 1000)
    conv[0]=(xx/1000)%10+'0';
  else
    conv[0]=' ';
  if (xx >= 100)
    conv[1]=(xx/100)%10+'0';
  else
    conv[1]=' ';
  if (xx >= 10)
    conv[2]=(xx/10)%10+'0';
  else
    conv[2]=' ';
  conv[3]=(xx)%10+'0';
  conv[4]=0;
  return conv;
}

//  convert float to string with 12345 format
char *ftostr5(const float &x)
{
  long xx=abs(x);
  if (xx >= 10000)
    conv[0]=(xx/10000)%10+'0';
  else
    conv[0]=' ';
  if (xx >= 1000)
    conv[1]=(xx/1000)%10+'0';
  else
    conv[1]=' ';
  if (xx >= 100)
    conv[2]=(xx/100)%10+'0';
  else
    conv[2]=' ';
  if (xx >= 10)
    conv[3]=(xx/10)%10+'0';
  else
    conv[3]=' ';
  conv[4]=(xx)%10+'0';
  conv[5]=0;
  return conv;
}

//  convert float to string with +1234.5 format
char *ftostr51(const float &x)
{
  long xx=x*10;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]=(xx/10)%10+'0';
  conv[5]='.';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

//  convert float to string with +123.45 format
char *ftostr52(const float &x)
{
  for(int i = 0; i < 10; i++) conv[i] = 0;
  long xx=x*100;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]='.';
  conv[5]=(xx/10)%10+'0';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

//  convert float to string with +123.4567 format
char *ftostr74(const float &x)
{
  long xx=x*10000;
  conv[0]=(xx>=0)?'+':'-';
  xx=abs(xx);
  conv[1]=(xx/1000000)%10+'0';
  conv[2]=(xx/100000)%10+'0';
  conv[3]=(xx/10000)%10+'0';
  conv[4]='.';
  conv[5]=(xx/1000)%10+'0';
  conv[6]=(xx/100)%10+'0';
  conv[7]=(xx/10)%10+'0';
  conv[8]=(xx)%10+'0';
  conv[9]=0;
  return conv;
}

//  convert float to string with +12345 format
char *ftostr6(const float &x)
{
  long xx=abs(x);
  if (xx >= 10000)
    conv[0]=(xx/10000)%10+'0';
  else
    conv[0]=(xx>=0)?'+':'-';
  if (xx >= 1000)
    conv[1]=(xx/1000)%10+'0';
  else
    conv[1]=' ';
  if (xx >= 100)
    conv[2]=(xx/100)%10+'0';
  else
    conv[2]=' ';
  if (xx >= 10)
    conv[3]=(xx/10)%10+'0';
  else
    conv[3]=' ';
  conv[4]=(xx)%10+'0';
  conv[5]=0;
  return conv;
}

// Callback for after editing PID i value
// grab the pid i value out of the temp variable; scale it; then update the PID driver
void copy_and_scalePID_i()
{
  Ki = scalePID_i(raw_Ki);
  updatePID();
}

// Callback for after editing PID d value
// grab the pid d value out of the temp variable; scale it; then update the PID driver
void copy_and_scalePID_d()
{
  Kd = scalePID_d(raw_Kd);
  updatePID();
}

#endif //ULTRA_LCD
