/**
 ******************************************************************************
* @file           : timers.h
* @author         : Muthuu SVS
* @brief          : relative timers, SysTick initialization, and API to timer system
******************************************************************************
*/

#ifndef TIMERS_H
#define TIMERS_H
#include "log.h"//include for debug logging
#include <stdint.h>

#define TICKTIME_MS (50U)//50 ticks for 1 millisecond
#define TICKS_US (20U)//1 tick = 20 microseconds
#define TICKMS_CONV(x) (x/TICKTIME_MS)//macro for determining ticks per ms

#define ONE_SECOND (1000)
#define ONE_SECOND_TICKS (TICKMS_CONV(ONE_SECOND))

#define QUARTER_SECOND_TICKS (TICKMS_CONV(250))
#define HALF_SECOND_TICKS (TICKMS_CONV(500))

#define TIMER_START_ID (0)//timer start marker
#define TIMER_START_ULED_ID (2)// uled timer start marker
#define TIMER_SOIL_SAMPLING_ID (3)// soil sampling timer start marker

typedef uint32_t ticktime_t;//time since boot, in 50ms increments

/*
* init the timing system for sysTick
* @param None
* @return None
* Reference:
* Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 212
*/
void init_systick(void);//init the timing system

/*
* returns time since startup in 1/16ths of a second
* @param none
* @return ticktime_t ticks how long as it been since startup
* Reference:
*/
ticktime_t now(void);

/*
* resets timer to 0, doesnt affect now() function values
* uses file scoped static variable timer_start which is a reference to make get_timer relative
* @param uint8_t timer, which timer to reset,
* @return none
* Reference:
*/
void reset_timer(uint8_t timer);

/*
* uses file scoped variable timer variable as reference point compare against now()
* for current time since reset_timer
* @param uint8_t timer which timer to get time for 
* @return ticks since last call to reset timer
* Reference:
*/
ticktime_t get_timer(uint8_t timer);

/*
* delays for a specified number of milliseconds
* @param uint32_t ms, number of milliseconds to delay
* @return none
* Reference:
*/
void delay_ms(uint32_t ms);

/* hard spin loop
* delays for a specified number of microseconds
* @param uint32_t us, number of microseconds to delay
* @return none
* Reference:
*/
void delay_us(uint32_t us);


#endif
