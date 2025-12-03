/**
 ******************************************************************************
 * @file           : state_machine.h
 * @author         : Muthuu SVS
 * @brief          : state_machine switching logic for overall system, #defines, etc
 ******************************************************************************
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include "log.h"//debug 

//normal sample rate: every 30 seconds
//watering sample rate every 300-500ms its a small plant
//should never reach dying of thirst levels
//too much water is also bad
//ideal is approx wet range
//calibrated thresholds based on real life values
#define DYING_OF_THIRST_THRESHOLD        (500)//Bone dry soil
#define DRY_THRESHOLD                    (1000)//acceptable   
#define WET_THRESHOLD                    (2100)//Damp soil after watering
#define SOAKING_THRESHOLD                (2800)//PURE Water Calibration

//states for watering state machine
typedef enum {
    WATER_IDLE = 0,
    WATER_MEASURE,
    WATER_WAIT_MEASURE,
    WATER_DECIDE,
    WATER_OPEN_VALVE,
    WATER_WATERING,
    WATER_CLOSE_VALVE
} water_state_t;


/* Helper function to get textual name for states for logging
 * @param none 
 * @return none
 * Reference : 
 */
const char* water_state_to_string(water_state_t state);

/* initializes state machine and servo angle, and starting sample rate
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_init(void);

/* watering state machine changes states and logs transitions
 * executes each state function, moving the servo, measuring, etc.
 * @param none 
 * @return none
 */
void water_fsm_run(void);



#endif//state_machine_h
