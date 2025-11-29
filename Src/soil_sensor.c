/**
 ******************************************************************************
 * @file           : soil_sensor.h
 * @author         : Muthuu SVS
 * @brief          : functions that control power initialization and reading values
 *                   for Soil Moisture Sensor such that sensor corrosion is minimized,
 * and power consumption is reduced.
 ******************************************************************************
 */

#include "soil_sensor.h"
#include "adc.h"        // for adc_manual_sample()
#include "timers.h"  // for power stablization timing
#include "log.h"        // for LOG()
#include <stm32f091xc.h>

//soil sensor state variable
static soil_state_t soil_state = SOIL_IDLE;

// Power pin: PA6
#define SOIL_PWR_PORT  (GPIOA)
#define SOIL_PWR_PIN   (6U)

// Stabilization time
#define SOIL_STABILIZE_MS     (50U)
#define SOIL_STABILIZE_TICKS  (TICKMS_CONV(SOIL_STABILIZE_MS))

// Calibration values (replace with real-world data)
#define SOIL_CALIB_DRY_RAW    (1U)
#define SOIL_CALIB_WET_RAW    (4095U)

#define MIN_PERCENTAGE (0U)
#define MAX_PERCENTAGE (100U)

//file scoped variables
//soil sensor values
static volatile uint16_t soil_raw;
static volatile uint8_t  soil_pct;

/* controls power to soil moisture sensor
 * @param state, ON or OFF equivalent to 1 or 0
 * @return none
 * Reference :
 */
void set_soil_power(uint8_t state)
{
    //turn on power to soil moisture sensor
    if(ON == state)
    {
        SOIL_PWR_PORT->ODR |= (1U << SOIL_PWR_PIN);
    }
    else//turn off power to soil moisture sensor
    {
        SOIL_PWR_PORT->ODR &= ~(1U << SOIL_PWR_PIN);
    }
}


/* convertes raw ADC value to percentage based on calibration sensor values
 * @param none 
 * @return none
 * Reference : 
 */
static uint8_t soil_raw_to_pct(uint16_t raw)
{
    //clamp values
    if (raw <= SOIL_CALIB_DRY_RAW) 
    {
        return MIN_PERCENTAGE;
    }
    if (raw >= SOIL_CALIB_WET_RAW) 
    {
        return MAX_PERCENTAGE;
    }
    //returns converted percentage  
    return (uint8_t)(((uint32_t)(raw - SOIL_CALIB_DRY_RAW) * MAX_PERCENTAGE) /(SOIL_CALIB_WET_RAW - SOIL_CALIB_DRY_RAW));
}

/* initializes GPIO to control power to soil moisture sensor
 * @param none 
 * @return none
 * Reference : 
 */
void init_soil_sensor_power(void)
{
    // Enable peripheral clock of GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Configure PA6 as digital output to control power to soil moisture sensor
    // (ESF_GPIO_MODER_OUTPUT = 1)
    MODIFY_FIELD(SOIL_PWR_PORT->MODER, GPIO_MODER_MODER6, ESF_GPIO_MODER_OUTPUT);
    
    //default off for sensor power supply
    set_soil_power(OFF);
}

/* initializes both ADC and GPIO for soil moisture sensor
 * and sensor state machine
 * @param none 
 * @return none
 * Reference : 
 */
void soil_sensor_init(void)
{
    init_ADC();
    init_soil_sensor_power();
    soil_state = SOIL_IDLE;
}

/* starts the soil moisture sensor measurement process
 * non blocking function to move soil sensor state machine
 * @param 
 * @return none
 * Reference : 
 */
void soil_sensor_begin_measurement(void)
{
    if (soil_state == SOIL_WAIT)
    {
        return; // measurement already in progress
    }
    //start measurement process
    set_soil_power(ON);//turns on power to soil moisture sensor
    reset_timer(TIMER_SOIL_SAMPLING_ID);//resests timer for stabilization timing
    soil_state = SOIL_WAIT;//moves state machine   
}

/* non-blocking state machine to handle soil moisture sensor measurement
 * will control power to sensor and take measurements based on soil_state
 * because sensor needs time to stabilize after power on
 * @param 
 * @return none
 * Reference : 
 */
void soil_measure_fsm(void)
{
    switch (soil_state)
    {
        case SOIL_IDLE:
            //nothing to do
            break;
        case SOIL_WAIT:
            if (get_timer(TIMER_SOIL_SAMPLING_ID) >= SOIL_STABILIZE_TICKS)
            {
                soil_raw = adc_manual_sample();//take ADC sample
                soil_pct = soil_raw_to_pct(soil_raw);//convert to percentage
                set_soil_power(OFF);//turns off power to soil moisture sensor
                soil_state = SOIL_DONE;
            }
            break;
        case SOIL_DONE:
            // reading available until next BeginMeasurement()
            break;
        default:
            //should not reach here
            LOG("Soil Sensor FSM Error: Invalid State %d\r\n", soil_state);
            soil_state = SOIL_IDLE;
            break;
    }
}

/* checks if soil moisture sensor measurement is done
 * based on state machine
 * @param 
 * @return none
 * Reference : 
 */
uint8_t soil_sensor_is_ready(void)
{
    return (soil_state == SOIL_DONE);
}

/* gets the raw ADC value from soil moisture sensor
 * @param 
 * @return none
 * Reference : 
 */
uint16_t soil_sensor_get_raw(void)
{
    return soil_raw;
}

/* gets the percentage moisture value from soil moisture sensor
 * @param 
 * @return none
 * Reference : 
 */
uint8_t soil_sensor_get_percent(void)
{
    return soil_pct;
}
