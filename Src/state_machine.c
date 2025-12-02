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

//servo defines
#define SERVO_OPEN_ANGLE     (270)
#define SERVO_CLOSED_ANGLE   (0)

//should never reach dying of thirst levels
//too much water is also bad
//ideal is approx wet range
#define DYING_OF_THIRST_THRESHOLD        (500)//Bone dry soil
#define DRY_THRESHOLD                    (1000)//acceptable   
#define WET_THRESHOLD                    (2100)//Damp soil after watering
#define SOAKING_THRESHOLD                (2800)//PURE Water Calibration

#define IDLE_SAMPLE_PERIOD_MS    (30000)   // 30 sec rate
#define WATER_SAMPLE_PERIOD_MS   (300)     // 30 ms rate fast sampling while watering

//Sample rate identifiers matches the periods
#define IDLE_SAMPLE_RATE_ID (0) 
#define WATERING_SAMPLE_RATE_ID (1)

//file scoped variables
static water_state_t state;//cur state
static water_state_t prev;//prev state
static ticktime_t next_sample_time;//next time to sample soil

/* Helper function to get textual name for states for logging
 * @param none 
 * @return none
 * Reference : 
 */
const char* water_state_to_string(water_state_t state)
{
    switch (state)
    {
        case WATER_IDLE: 
            return "WATER_IDLE";
            break;
        case WATER_MEASURE: 
            return "WATER_MEASURE";
            break;
        case WATER_WAIT_MEASURE: 
            return "WATER_WAIT_MEASURE";
            break;
        case WATER_DECIDE: 
            return "WATER_DECIDE";
            break;
        case WATER_OPEN_VALVE: 
            return "WATER_OPEN_VALVE";
            break;
        case WATER_WATERING: 
            return "WATER_WATERING";
            break;
        case WATER_CLOSE_VALVE: 
            return "WATER_CLOSE_VALVE";
            break;
        default: 
            LOG("error unknown state\r\n");
            return "UNKNOWN_STATE";
            break;
    }
}

/*
 * Schedule next sample time using one of two predefined intervals.
 * @param rate: WATERING_SAMPLE_RATE_ID to use WATER_SAMPLE_PERIOD_MS
 *              IDLE_SAMPLE_RATE_ID` to use `IDLE_SAMPLE_PERIOD_MS
 */
void schedule_next_sample(uint8_t rate)
{
    if (rate == WATERING_SAMPLE_RATE_ID)
    {
        next_sample_time = now() + (WATER_SAMPLE_PERIOD_MS / TICKTIME_MS);
    }
    else//always idle incase of error
    {
        next_sample_time = now() + (IDLE_SAMPLE_PERIOD_MS / TICKTIME_MS);
    }
}

/* initializes state machine and servo angle, and starting sample rate
 * @param none 
 * @return none
 * Reference : 
 */
void water_fsm_init(void)
{
    // Initialization code for the watering state machine
    LOG("Water FSM Initialized\r\n");
    state = WATER_IDLE;
    prev = state;
    LOG("Transition: %s -> %s\r\n", "UNKNOWN", water_state_to_string(state));
    // schedule initial idle sample
    schedule_next_sample(IDLE_SAMPLE_RATE_ID);
    servo_set_angle(SERVO_CLOSED_ANGLE);
}

/* watering state machine changes states and logs transitions
 * executes each state function, moving the servo, measuring, etc.
 * @param none 
 * @return none
 */
void water_fsm_run(void)
{
    ticktime_t current = now(); //gets current time
    water_state_t prev = state; // shared previous-state variable for logging

    switch (state) //state machine branch
    {
        case WATER_IDLE: //idle timing
            if (current >= next_sample_time) //if idle sample time is completed
            {
                soil_sensor_begin_measurement(); //start sampling the soil moisture level
                prev = state;
                state = WATER_WAIT_MEASURE; //move fsm to measuring state
                LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            }
            break;
        case WATER_WAIT_MEASURE:
            if (soil_sensor_is_ready()) //if power stabilized and ready
            {
                prev = state;
                state = WATER_DECIDE; //move to decision state
                LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            }
            break;
        case WATER_DECIDE:
        {
            uint16_t raw = soil_sensor_get_raw(); //gets value
            LOG("Soil Reading: %u\r\n", raw);

            //extreme values
            if (raw < DYING_OF_THIRST_THRESHOLD)
            {
                LOG("WARNING: Soil critically dry - possible sensor failure!\r\n");
            }
            else if (raw > SOAKING_THRESHOLD)
            {
                LOG("WARNING: Soil over-saturated - possible sensor failure!\r\n");
            }

            if (raw < DRY_THRESHOLD) //checks if soil is dry
            {
                LOG("Soil is dry -> start watering\r\n");
                prev = state;
                state = WATER_OPEN_VALVE;
                LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            }
            else
            {
                schedule_next_sample(IDLE_SAMPLE_RATE_ID);
                prev = state;
                state = WATER_IDLE;
                LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            }
        }
        break;
        case WATER_OPEN_VALVE:
            LOG("Opening valve!\r\n");
            servo_set_angle(SERVO_OPEN_ANGLE);
            soil_sensor_begin_measurement(); // start fast sampling
            schedule_next_sample(WATERING_SAMPLE_RATE_ID); //set fast sampling
            prev = state;
            state = WATER_WATERING; //move to water-the-plant state
            LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            break;
        case WATER_WATERING: //water the plant state
            if (soil_sensor_is_ready()) //check if soil ready for measurement
            {
                uint16_t raw = soil_sensor_get_raw();
                LOG("Watering reading: %u\r\n", raw); //print soil measurement while measuring

                if (raw > WET_THRESHOLD) //when wet, stop watering
                {
                    LOG("Soil is wet -> stopping water\r\n");
                    prev = state;
                    state = WATER_CLOSE_VALVE;
                    LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
                    break;
                }
                //start measurement again while watering
                soil_sensor_begin_measurement();
                //keep sampling time at fast rate
                schedule_next_sample(WATERING_SAMPLE_RATE_ID);
            }
            break;
        case WATER_CLOSE_VALVE: //close valve
            LOG("Closing valve...\r\n");
            servo_set_angle(SERVO_CLOSED_ANGLE);
            schedule_next_sample(IDLE_SAMPLE_RATE_ID); //go back to idle interval
            prev = state;
            state = WATER_IDLE;
            LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            break;
        default:
            prev = state;
            state = WATER_IDLE;
            LOG("Transition: %s -> %s\r\n", water_state_to_string(prev), water_state_to_string(state));
            break;
    }
}



