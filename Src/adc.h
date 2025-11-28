/**
 ******************************************************************************
 * @file           : adc.h
 * @author         : Muthuu SVS
 * @brief          : functions that control ADC initialization and reading values
 * for all sensors involved in the project: Soil Moisture Sensor.
 ******************************************************************************
 */

#ifndef ADC_H
#define ADC_H
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

/* returns the latest soil moisture sensor value
 * @param none 
 * @return uint16_t soil_moisture_value
 * Reference : 
 */
uint16_t get_soil_moisture_value(void);

/* returns the latest soil moisture sensor value
 * @param none 
 * @return uint16_t soil_moisture_value
 * Reference : 
 */
uint16_t adc_manual_sample(void);

#endif /* ADC_H */
