/**
 ******************************************************************************
 * @file           : adc.h
 * @author         : Muthuu SVS
 * @brief          : functions that control ADC initialization and reading values
 * for all sensors involved in the project: Soil Moisture Sensor.
 ******************************************************************************
 */

#ifndef PWM_H
#define PWM_H
#include <utilities.h>

/*
 * initializes ADC1 for Soil Moisture Sensor
 * enabling gpio port and #bit to be an analog input
 * @param none using registers
 * @return none
 * Reference : Reference Manual RM0091 Chapter 10, Section 10.6.1
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, 
 * Chapter: 6 Analog-​to-​Digital Converter, Page No. 194
 */
void init_ADC(void);

 /*
 * initializes TIMER15 for ADC triggered sampling
 * @param none using registers
 * @return none
 * Reference : 
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, 
 * Chapter: 7 Timers, Page No. 212
 */
void init_TIM15(void);

/* sets the ARR value for ADC timer
 * @param uint32_t arr_value
 * @return none
 * Reference : 
 */
void set_adc_timer_arr(uint32_t arr_value);

/* calculates and sets the ARR value for ADC timer based on desired sample rate
 * @param uint32_t sample_rate in Hz 
 * @return none
 * set file scoped variable adc_timer_sample_rate
 * Reference : 
 */
void calculate_adc_timer_arr(uint32_t sample_rate);

/* returns the latest soil moisture sensor value
 * @param none 
 * @return uint16_t soil_moisture_value
 * Reference : 
 */
uint16_t get_soil_moisture_value(void);

#endif /* ADC_H */