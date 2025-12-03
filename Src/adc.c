/**
 ******************************************************************************
 * @file           : adc.c
 * @author         : Muthuu SVS
 * @brief          : functions that control ADC initialization and reading values
 * for all sensors involved in the project: Soil Moisture Sensor.
 ******************************************************************************
 */
 #include "adc.h"
 #include <stm32f091xc.h>
 #include "log.h"
 #include "utilities.h" // for ERROR_CODE


// ADC health flag and timeouts
static volatile uint8_t adc_ok = 0;
#define HSI14_READY_TIMEOUT    (100000U)
#define ADC_READY_TIMEOUT      (100000U)
#define ADC_EOC_TIMEOUT        (100000U)
//SOIL SENSOR Defines
#define SOIL_SENSOR_PORT (GPIOA)
#define SOIL_SENSOR_PIN  (0U) //PA0 -> ADC1_IN0
#define SOIL_SENSOR_CHANNEL (0U) //ADC1_IN0

#define SOIL_PWR_PORT (GPIOA)
#define SOIL_PWR_PIN  (6U) //PA6 -> Digital Output control of power soil moisture sensor

#define SOIL_STABILIZATION_TIME_MS (50U) //time for soil moisture sensor to stabilize after power on

 /*
 * initializes ADC1 for Soil Moisture Sensor
 * enabling gpio port and #bit to be an analog input
 * @param none using registers
 * @return none
 * Reference : Reference Manual RM0091 Chapter 10, Section 10.6.1
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, 
 * Chapter: 6 Analog-​to-​Digital Converter, Page No. 194
 */
void init_ADC(void)
{
    // Enable peripheral clock of ADC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    // Enable peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Configure PA0 as analog input to ADC
    // (ESF_GPIO_MODER_ANALOG = 3)
    MODIFY_FIELD(SOIL_SENSOR_PORT->MODER, GPIO_MODER_MODER0, ESF_GPIO_MODER_ANALOG);
    // Oscillator: Enable and Select HSI14 (from STM32F0 Reference Manual, A.7.4)
    RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) Start HSI14 RC oscillator */

    // Wait for HSI14 ready with timeout
    uint32_t t = 0;
    while (((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) && (t++ < HSI14_READY_TIMEOUT))
    {
        __NOP();
    }
    if ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0)
    {
        LOG("ADC Error: HSI14 oscillator timeout\r\n");
        adc_ok = 0;
        return;
    }

    // Ensure ADC is disabled before config
    if (ADC1->CR & ADC_CR_ADEN)
    {
        ADC1->CR |= ADC_CR_ADDIS;
    }
    // Wait for disable
    t = 0;
    while ((ADC1->CR & ADC_CR_ADEN) && (t++ < ADC_READY_TIMEOUT))
    {
        __NOP();
    }
    if (ADC1->CR & ADC_CR_ADEN)
    {
        LOG("ADC Error: could not disable ADC before config\r\n");
        adc_ok = 0;
        return;
    }

    //adc calibration
    ADC1->CR |= ADC_CR_ADCAL;
    t = 0;
    while ((ADC1->CR & ADC_CR_ADCAL) && (t++ < ADC_READY_TIMEOUT))
    {
        __NOP();
    }
    if (ADC1->CR & ADC_CR_ADCAL)
    {
        LOG("ADC Error: calibration timeout\r\n");
        adc_ok = 0;
        return;
    }

    ADC1->CFGR1 = 0;//reset CFGR1 register

    /* Select HSI14 with CKMODE=00 */
    MODIFY_FIELD(ADC1->CFGR2, ADC_CFGR2_CKMODE, 0);
    // Init ADCl
    MODIFY_FIELD(ADC1->SMPR, ADC_SMPR_SMP, 0); // SMP = 000 for minimum sample time

    /* CFGR1: The default configuration (CFGR1 = 0) matches what we want:
      some features are disabled (analog watchdog, discontinous conversion
      mode, auto-off mode, wait conversion mode, continuous conversion mode,
      hardware trigger) and other features are selected: software trigger,
      right-aligned data, 12-bit resolution. */
    ADC1->CFGR1 = 0;

    // Select ADC channel to convert
    ADC1->CHSELR = ADC_CHSELR_CHSEL0; // Select ADC input channel 0
    // Enable ADC (from STM32F0 Reference Manual, A.7.2
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
    { /* (1) Ensure that ADRDY = 0 */
        ADC1->ISR |= ADC_ISR_ADRDY; /* (2) Clear ADRDY */
    }

    ADC1->CR |= ADC_CR_ADEN; /* (3) Enable the ADC */

    // Wait for ADC ready with timeout
    t = 0;
    while (((ADC1->ISR & ADC_ISR_ADRDY) == 0) && (t++ < ADC_READY_TIMEOUT))
    {
        __NOP();
    }
    if ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
    {
        LOG("ADC Error: ADRDY timeout\r\n");
        adc_ok = 0;
        return;
    }

    // Mark ADC as ok
    adc_ok = 1;
}

/* returns the latest soil moisture sensor value
 * @param none 
 * @return uint16_t soil_moisture_value
 * Reference : 
 */
uint16_t adc_manual_sample(void)
{
    if(!adc_ok)
    {
        LOG("ADC Error: sample requested but ADC not initialized\r\n");
        return ERROR_CODE;
    }

    if(ADC1->ISR & ADC_ISR_EOC)
    {
        ADC1->ISR |= ADC_ISR_EOC; //clear EOC flag
    }

    //start conversion
    ADC1->CR |= ADC_CR_ADSTART;

    //wait for conversion to complete with timeout
    uint32_t t = 0;
    while(((ADC1->ISR & ADC_ISR_EOC) == 0) && (t++ < ADC_EOC_TIMEOUT))
    {
        __NOP();
    }
    if ((ADC1->ISR & ADC_ISR_EOC) == 0)
    {
        LOG("ADC Error: conversion EOC timeout\r\n");
        return ERROR_CODE;
    }

    return ADC1->DR; //read converted value

}

/* Return ADC initialized state */
uint8_t adc_initialized(void)
{
    return adc_ok;
}

