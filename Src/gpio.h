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
#define ELED_IDENTITY (7U)

/*
 * initializes eled function
 * enabling gpio port and #bit to be an output
 * @param none using registers
 * @return none
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void init_eled(void);

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
 * set ELED function
 * @param state ON or OFF
 * @return no return function
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_eled(uint8_t state);

/*
 * set ULED function
 * @param state ON or OFF
 * @return no return function
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_uled(uint8_t state);

/*
 * initializes button function
 * enabling gpio port and #bit to be an output
 * @param none using registers
 * @return none
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 * Reference PES 10 F091RC and GPIO pt2.pdf
 */
void init_button(void);

/*
 * get button switch state function
 * @param none using registers
 * @return button pressed (1) or not (0)
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 * Reference PES 10 F091RC and GPIO pt2.pdf
 */
uint8_t get_button_state(void);

/*
* Initializes gpio port peripheral and configures TIMER3 and channel 2 counter
* @param none
* @return none
* Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 232
*/
void init_PWM_eled(void);

/*
* sets static brightness level that is used by PWM TIMER for ELED brightness 
* @param uint16_t value to update pwm brightness level uint8_t led identity what led brightness to set
* @return none
* Reference: none
*/
void set_brightness_scale(uint16_t value, uint8_t led_identity);

/*
* gets static brightness level that is used by PWM TIMER for ELED brightness 
* @param none
* @return uint16_t value to update pwm brightness level
* Reference: none
*/
uint16_t get_brightness_scale(void);

#endif
