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


//ADC defines

//TIMER 15 defines
#define ADC_TIMER (TIM15)
#define ADC_TIMER_PRESCALER (48 - 1)
#define ADC_TIMER_CLK_HZ (F_TIM_CLOCK / (ADC_TIMER_PRESCALER + 1))// 1 MHz clk

//SOIL MOISTURE SENSOR defines
#define SOIL_SENSOR_PORT (GPIOA)
#define SOIL_SENSOR_PIN  (0) //PA0 -> ADC1_IN0
#define SOIL_SENSOR_CHANNEL (0) //ADC1_IN0

//file scope variables
volatile uint32_t soil_moisture_value = 0;
volatile uint32_t adc_timer_sample_rate = 0;

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
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0)
    { /* (3) Wait HSI14 is ready */
    /* For robust implementation, add here time-out management */
    }
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
    ADC1->CFGR1 |= (0b011 << ADC_CFGR1_EXTSEL_Pos); //select TIM15_TRGO event as external trigger
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;//rising edge 

    // Select ADC channel to convert
    ADC1->CHSELR = ADC_CHSELR_CHSEL0; // Select ADC input channel 0
    // Enable ADC (from STM32F0 Reference Manual, A.7.2
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
    { /* (1) Ensure that ADRDY = 0 */
        ADC1->ISR |= ADC_ISR_ADRDY; /* (2) Clear ADRDY */
    }
    ADC1->IER |= ADC_IER_EOCIE; /* Enable end of conversion interrupt */
    NVIC_EnableIRQ(ADC1_COMP_IRQn); // Enable ADC interrupt in NVIC

    ADC1->CR |= ADC_CR_ADEN; /* (3) Enable the ADC */
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) 
    { /* (4) Wait until ADC ready */
    /* For robust implementation, add here time-out management */
    }

    // Start ADC external-trigger conversions
    ADC1->CR |= ADC_CR_ADSTART;
}

 /*
 * initializes TIMER15 for ADC triggered sampling
 * @param none using registers
 * @return none
 * Reference : 
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, 
 * Chapter: 7 Timers, Page No. 212
 */
void init_TIM15(void)
{
    // Enable peripheral clock of TIM15
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    // Configure TIM15 for triggering ADC at regular intervals
    ADC_TIMER->PSC = ADC_TIMER_PRESCALER; // Prescaler value 
    ADC_TIMER->ARR = 10000; // temp arr value, will be updated later
    
    ADC_TIMER->CR2 &= ~TIM_CR2_MMS; // Clear MMS bits
    ADC_TIMER->CR2 |= TIM_CR2_MMS_1; // Update event as trigger output (TRGO)
    ADC_TIMER->CR1 |= TIM_CR1_CEN; // Enable TIM15
}

/* sets the ARR value for ADC timer
 * @param uint32_t arr_value
 * @return none
 * Reference : 
 */
void set_adc_timer_arr(uint32_t arr_value)
{
    ADC_TIMER->ARR = arr_value;
}

/* calculates and sets the ARR value for ADC timer based on desired sample rate
 * @param uint32_t sample_rate in Hz 
 * @return none
 * set file scoped variable adc_timer_sample_rate
 * Reference : 
 */
void calculate_adc_timer_arr(uint32_t sample_rate)
{
    uint32_t arr_value = (ADC_TIMER_CLK_HZ / sample_rate) - 1;
    set_adc_timer_arr(arr_value);
    adc_timer_sample_rate = sample_rate;
}

/* ADC1 interrupt handler to convert data to soil_moisture_value
 * @param none 
 * @return none
 * Reference : 
 */
void ADC1_COMP_IRQHandler(void)
{
    if ((ADC1->ISR & ADC_ISR_EOC) != 0)
    {
        // Read the converted value
        soil_moisture_value = ADC1->DR; // Reading DR clears EOC flag
    }
}

/* returns the latest soil moisture sensor value
 * @param none 
 * @return uint16_t soil_moisture_value
 * Reference : 
 */
uint32_t get_soil_moisture_value(void)
{
    return soil_moisture_value;
}
