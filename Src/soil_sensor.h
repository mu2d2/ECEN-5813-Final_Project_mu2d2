/******************************************************************************
 * @file           : soil_sensor.h
 * @author         : Muthuu SVS
 * @brief          : functions that control power initialization and reading values
 *                   for Soil Moisture Sensor such that sensor corrosion is minimized,
 * and power consumption is reduced.
 ******************************************************************************
 */
#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include <stdint.h>

//sensor power states
typedef enum {
    SOIL_IDLE = 0,//sensor off idle
    SOIL_WAIT,//waiting for stabilization
    SOIL_DONE//measurement done
} soil_state_t;

/* initializes both ADC and GPIO for soil moisture sensor
 * and sensor state machine
 * @param none 
 * @return none
 * Reference : 
 */
void soil_sensor_init(void);

/* starts the soil moisture sensor measurement process
 * non blocking function to move soil sensor state machine
 * @param 
 * @return none
 * Reference : 
 */
void soil_sensor_begin_measurement(void);

/* non-blocking state machine to handle soil moisture sensor measurement
 * will control power to sensor and take measurements based on soil_state
 * because sensor needs time to stabilize after power on
 * @param 
 * @return none
 * Reference : 
 */
void soil_measure_fsm(void);

/* checks if soil moisture sensor measurement is done
 * based on state machine
 * @param 
 * @return none
 * Reference : 
 */
uint8_t  soil_sensor_is_ready(void);
/* gets the raw ADC value from soil moisture sensor
 * @param 
 * @return none
 * Reference : 
 */
uint16_t soil_sensor_get_raw(void);
/* gets the percentage moisture value from soil moisture sensor
 * @param 
 * @return none
 * Reference : 
 */
uint8_t  soil_sensor_get_percent(void);

/* converts raw ADC value to percentage based on calibration sensor values
 * @param raw ADC value
 * @return percentage moisture (0-100)
 */
uint8_t soil_raw_to_pct(uint16_t raw);

#endif
