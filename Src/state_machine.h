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

//normal sample rate: every 60 seconds

//watering sample rate every 300-500ms its a small plant


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


/*
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_init(void);

/*
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_run(void);



#endif//state_machine_h
