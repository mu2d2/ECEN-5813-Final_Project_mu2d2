/**
 ******************************************************************************
 * @file           : gpio.h
 * @author         : Muthuu SVS
 * @brief          : functions that manipulate the gpio, initializing registers
 *  for buttons, Leds, and setting buttons, and contains PWM initialization and interrupt handler
 ******************************************************************************
 */

#ifndef GPIO_H
#define GPIO_H
#include <stm32f091xc.h>
#include <utilities.h>

//defines used for ledIdentity
#define ULED_IDENTITY (5U)
#define SERVO_IDENTITY (7U)

/*
 * initializes uled function
 * enabling gpio port and #bit to be an output
 * @param none using registers
 * @return none
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void init_uled(void);

/*
 * set ULED function
 * @param state ON or OFF
 * @return no return function
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_uled(uint8_t state);

/*
 * set SERVO function
 * @param state ON or OFF
 * @return no return function
 * 
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_servo(uint8_t state);

/*
* Initializes gpio port peripheral and configures TIMER3 and channel 2 counter
* @param none
* @return none
* Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 232
*/
void init_PWM_SERVO(void);

/*
 * @param 
 * @return 
 * 
 */
void servo_set_angle(uint16_t angle);


#endif
