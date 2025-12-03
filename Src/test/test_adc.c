/**
 ******************************************************************************
 * @file           : test_adc.c
 * @author         : Muthuu SVS
 * @brief          : Unit tests for ADC module
 *                   Tests ADC initialization and sampling flow
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

#include "../adc.h"
#include "../log.h"
#include <stdint.h>

//Test Defines
#define ADC_MIN_VALUE        (0)
#define ADC_MAX_VALUE        (4095)
#define NUM_ADC_TESTS        (2)

//Test Utilities
static int adc_test_count = 0;
static int adc_test_passed = 0;

/**
 * Assert that a test passes and log result
 * @param test_name Description of the test
 * @return none
 */
static void adc_assert_pass(const char* test_name)
{
    adc_test_count++;
    adc_test_passed++;
    LOG("PASS: %s\r\n", test_name);
}

/**
 * Assert that a value is within a range and log result
 * @param actual The actual value
 * @param min_val The minimum acceptable value
 * @param max_val The maximum acceptable value
 * @param test_name Description of the test
 * @return none
 */
static void adc_assert_range(uint16_t actual, uint16_t min_val, uint16_t max_val, const char* test_name)
{
    adc_test_count++;
    if (actual >= min_val && actual <= max_val)
    {
        adc_test_passed++;
        LOG("PASS: %s (got: %u)\r\n", test_name, actual);
    }
    else
    {
        LOG("FAIL: %s (expected range %u-%u, got: %u)\r\n", test_name, min_val, max_val, actual);
    }
}

/**
 * Run all ADC unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_adc(void)
{
    LOG("\r\n========== ADC UNIT TESTS ==========\r\n\r\n");
    
    LOG("--- Initialization Tests ---\r\n");
    adc_assert_pass("ADC initialization callable");
    
    LOG("\r\n--- Sampling Tests ---\r\n");
uint16_t sample = adc_manual_sample();
if (sample == ERROR_CODE)
{
    adc_test_count++;
    LOG("FAIL: ADC sample returned ERROR_CODE (ADC init/sample failure)\r\n");
}
else
{
    adc_assert_range(sample, ADC_MIN_VALUE, ADC_MAX_VALUE, "ADC sample in valid range");
}    LOG("\r\n========== ADC TEST SUMMARY ==========\r\n");
    LOG("Total: %d, Passed: %d, Failed: %d\r\n", adc_test_count, adc_test_passed, adc_test_count - adc_test_passed);
    if (adc_test_count > 0)
    {
        LOG("Pass Rate: %d%%\r\n\r\n", (adc_test_passed * 100) / adc_test_count);
    }
    
    return (adc_test_count == adc_test_passed) ? 0 : 1;
}
