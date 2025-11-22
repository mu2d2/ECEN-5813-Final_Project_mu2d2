/**
 ******************************************************************************
 * @file           : state_machine.h
 * @author         : Muthuu SVS
 * @brief          : state_machine switching logic, #defines, etc
 ******************************************************************************
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include "timers.h"//for timer access
#include "gpio.h"//for setting uled access
#include "log.h"//debug 

//State Machine for BUS
typedef enum busStateMachine
{
    BOULDER_STATE,
    LAFAYETTE_STATE,
    DENVER_STATE,
    TRANS_STATE,
    E_STOP_STATE
}BusState;

//State Machine for Blink Status
typedef enum ledStateMachine
{
    LED_STATE_OFF,
    LED_BLINK,
    LED_FADE,
    LED_HOLD,
}ledState;

//typedef for led struct, toggle, and led state machine
typedef struct ledStruct
{
    ledState prevLedState;
    ledState thestate;
    uint8_t identity;//eled or uled
    uint8_t led_brightness;
    uint8_t target_brightness;
    uint8_t blinker_count;
    uint8_t led_toggle;
    uint8_t pauseFlag;
    uint8_t prev_brightness;//last stable brightness before transition
    int16_t fade_step;// +/- direction for fade step of brightness
}ledStatus;

/*
 * initializes led status structs for ELED and ULED
 * edits file scoped led static variables
 * @param none 
 * @return none
 * Reference : 
 */
void init_led_status(void);

/*
 * updates State Machine of where the Bus is
 * uses function scoped BusState variables ucBusLine and nextState
 * @param none 
 * @return none
 * Reference : 
 */
void updateBusState(void);

/*
 * on first entry to led state reset led timer
 * @param ledStatus *xled  pointer to the led to check for first entry led states
 * @return none
 * Reference : 
 */
void first_entry_led(ledStatus *xled);

/*
 * On first entry of the state prints DEBUG message, and resets timer for state and updates prev state
 * uses file scoped bus State variables
 * @param none 
 * @return none
 * Reference : 
 */
void first_entry(void);

/* Executing State Machine Logic, setting brightness of LED
 * blinking status state for ELED and ULED,
 * and starting timers for each state
 * also setting next state & stop state for updating 
 * uses filed scoped bus state variables
 * @param none 
 * @return none
 * Reference : 
 */
void state_logic(void);

/*E_STOP HANDLER
 *sends State Machine to E-STOP from checking button state
 *resets e stop timer if repressed before 5 seconds is over
 *uses file scoped bus State variables
 * @param none 
 * @return none
 * Reference : 
 */
void estop_handler(void);

/*LED handler
 * timing for LED blinking, pausing, and fading logic, access led timers depending on state and LED
 * @param BusState bus for determining brightness fading, ledStatus *led to determine which led to change
 * @return none
 * Reference : 
*/
void led_handler(BusState bus, ledStatus *led);


#endif//state_machine_h
