/**
 ******************************************************************************
 * @file           : state_machine.h
 * @author         : Muthuu SVS
 * @brief          : state_machine switching logic for overall system, #defines, etc
 ******************************************************************************
 */

#include "state_machine.h"
#include "timers.h"//for timer access
#include "pwm.h"//for setting uled access and servo control
#include "soil_sensor.h"//for soil sensor state machine
#include "log.h"//debug

/*
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_init(void)
{
    // Initialization code for the watering state machine
    LOG("Water FSM Initialized\r\n");
}

/*
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_run(void)
{
    LOG("Water FSM Running\r\n");
}



