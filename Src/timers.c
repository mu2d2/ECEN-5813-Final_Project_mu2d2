/**
 ******************************************************************************
* @file           : timers.c
* @author         : Muthuu SVS
* @brief          : Timers, SysTick initialization, and API to timer system
******************************************************************************
*/

#include "timers.h"
#include <stm32f091xc.h>
#include <utilities.h>
#include "core_cm0.h"//include for sysTick
//SYS TICK definitions
#define F_SYS_CLK (48000000L)//sys clock at 48MHz
#define TICK_TIME_CLK (20U)// to measure 1 timer tick as 50ms, need 20Hz interrupt
#define SYS_TICK_INT_PRIOR (3U)//systick interrupt priority

//TIMER VARIABLES
static ticktime_t total_ticks = 0;//SysTick Counter every interrupt which is every 20Hz aka 50ms
ticktime_t timer_start;//relative timer start marker
ticktime_t uled_timer_start;//relative timer start marker for blink seq control for uled
ticktime_t soil_sampling_timer_start;//relative timer start marker for soil sampling control

/*
* init the timing system for sysTick
* @param None
* @return None
* Reference:
* Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 212
*/
void init_systick(void)
{
    //  SysTick is defined in core_ cm0.h
    //  Set reload field to get 50ms interrupts aka 20Hz
    MODIFY_FIELD(SysTick->LOAD, SysTick_LOAD_RELOAD, (F_SYS_CLK/TICK_TIME_CLK));
    //  Set interrupt priority
    NVIC_SetPriority(SysTick_IRQn, SYS_TICK_INT_PRIOR);
    //  Force load of reload value
    MODIFY_FIELD(SysTick->VAL, SysTick_VAL_CURRENT, 0);
    //  use core clock 48MHz dont need to go to alt clk src Enable interrupt, enable SysTick timer
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    
    //initalizes start timers
    timer_start = total_ticks;
    uled_timer_start = total_ticks;
    soil_sampling_timer_start = total_ticks;
}

/* SysTick Interrupt Service Routine
 * Increments timer tick(50ms)
 * @param none
 * @return none
 * Reference:
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 213
 */
void SysTick_Handler(void)
{
    total_ticks++;
}

/*
* returns time since startup in 50ms increments
* @param none
* @return ticktime_t ticks how long as it been since startup
* Reference:
*/
ticktime_t now(void)
{
    return total_ticks;
}

/*
* resets timer to 0, doesnt affect now() function values
* uses file scoped static variables timer_start which is a reference to make get_timer relative
* @param uint8_t timer, which timer to reset
* @return none
* Reference:
*/
void reset_timer(uint8_t timer)
{
    switch (timer)
    {
        case TIMER_START_ID:
            timer_start = now();
            break;
            break;
        case TIMER_START_ULED_ID:
            uled_timer_start = now();
            break;
        case TIMER_SOIL_SAMPLING_ID:
            soil_sampling_timer_start = now();
            break;
        default:
            LOG("reset_timer(): Invalid timer ID\r\n");
            break;
    }
}

/*
* uses file scoped variable timer variable as reference point compare against now()
* for current time since reset_timer
* @param uint8_t timer which timer to get time for 
* @return ticks since last call to reset timer
* Reference:
*/
ticktime_t get_timer(uint8_t timer)
{
    switch (timer)
    {
        case TIMER_START_ID:
            return (now() - timer_start);
        case TIMER_START_ULED_ID:
            return (now() - uled_timer_start);
        case TIMER_SOIL_SAMPLING_ID:
            return (now() - soil_sampling_timer_start);
        default:
            LOG("get_timer(): Invalid timer ID\r\n");
            return ERROR;
    }
    
}

/*
* delays for a specified number of microseconds
* @param uint32_t us, number of microseconds to delay
* @return none
* Reference:
*/
void delay_us(uint32_t us)
{

}