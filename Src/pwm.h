/**
 ******************************************************************************
 * @file           : pwm.h
 * @author         : Muthuu SVS
 * @brief          : functions that manipulate the gpio, initializing registers
 *  for leds, and contains PWM initialization and interrupt handler for servo motor
 ******************************************************************************
 */

#ifndef PWM_H
#define PWM_H
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

/* sets the position of the servo motors based on angle input
 * assigns the target_ccr variable to the converted angle value
 * @param uint16_t angle desired angle between 0 and 270 degrees
 * @return none
 */
void servo_set_angle(uint16_t angle);


#endif
