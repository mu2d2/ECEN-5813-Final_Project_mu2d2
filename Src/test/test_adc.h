/**
 ******************************************************************************
 * @file           : test_adc.h
 * @author         : Muthuu SVS
 * @brief          : Unit tests for ADC module
 *
 * Hardware: These tests call `adc_manual_sample()` and therefore require
 *           the ADC input used by the project to be physically connected
 *           to a stable voltage or the sensor under test. For meaningful
 *           results, connect the expected sensor output or a known test
 *           voltage (0 - VREF, typically 0-3.3V) to the ADC channel.
 *           If the ADC input is floating or the sensor is disconnected,
 *           samples may be unpredictable and tests may fail or report
 *           unexpected values. Perform an in-person wiring check if
 *           hardware connectivity is in question.
 ******************************************************************************
 */

#ifndef TEST_ADC_H
#define TEST_ADC_H

/**
 * Run all ADC unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_adc(void);

#endif /* TEST_ADC_H */
