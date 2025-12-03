/**
 ******************************************************************************
 * @file           : test_soil_sensor.h
 * @author         : Muthuu SVS
 * @brief          : Unit tests for soil sensor module
 *
 * Hardware: These tests exercise only the raw-to-percentage conversion
 *           logic and do not perform ADC sampling. No hardware wiring is
 *           required to run these unit tests. To validate the actual
 *           soil sensor wiring and ADC integration, use `test_adc.h`
 *           or run the full application on hardware with the sensor
 *           connected to the ADC input.
 ******************************************************************************
 */

#ifndef TEST_SOIL_SENSOR_H
#define TEST_SOIL_SENSOR_H

/**
 * Run all soil sensor unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_soil_sensor(void);

#endif /* TEST_SOIL_SENSOR_H */
