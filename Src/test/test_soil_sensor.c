/**
 ******************************************************************************
 * @file           : test_soil_sensor.c
 * @author         : Muthuu SVS
 * @brief          : Unit tests for soil sensor module
 *                   Tests soil_raw_to_pct() conversion logic and edge cases
 *
 * Hardware: These tests exercise only the raw-to-percentage conversion
 *           logic and do not perform ADC sampling. No hardware wiring is
 *           required to run these unit tests. To validate the actual
 *           soil sensor wiring and ADC integration, use `test_adc.c`
 *           or run the full application on hardware with the sensor
 *           connected to the ADC input.
 ******************************************************************************
 */

#include "../soil_sensor.h"
#include "../log.h"
#include <stdint.h>

//Test Defines
#define SOIL_RAW_MIN          (1U)
#define SOIL_RAW_MAX          (4095U)
#define MIN_PERCENTAGE        (0U)
#define MAX_PERCENTAGE        (100U)
#define NUM_SOIL_TESTS        (5)

//Test Vectors
typedef struct {
    uint16_t raw_value;
    uint8_t expected_pct;
    const char* description;
} soil_test_vector_t;

static const soil_test_vector_t soil_test_vectors[NUM_SOIL_TESTS] = {
    { 0,    0,   "Lower clamp: 0" },
    { 1,    0,   "Lower clamp: at DRY_RAW boundary" },
    { 2048, 50,  "Midpoint 50 percent" },
    { 4095, 100, "Upper clamp: at WET_RAW boundary" },
    { 4096, 100, "Upper clamp: above WET_RAW" },
};

//Test Utilities
static int soil_test_count = 0;
static int soil_test_passed = 0;

/**
 * Assert that two uint8 values are equal and log result
 * @param actual The actual value
 * @param expected The expected value
 * @param test_name Description of the test
 * @return none
 */
static void soil_assert_equal(uint8_t actual, uint8_t expected, const char* test_name)
{
    soil_test_count++;
    if (actual == expected)
    {
        soil_test_passed++;
        LOG("PASS: %s (expected: %u, got: %u)\r\n", test_name, expected, actual);
    }
    else
    {
        LOG("FAIL: %s (expected: %u, got: %u)\r\n", test_name, expected, actual);
    }
}

/**
 * Run all soil sensor unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_soil_sensor(void)
{
    LOG("\r\n========== SOIL SENSOR UNIT TESTS ==========\r\n\r\n");
    
    for (int i = 0; i < NUM_SOIL_TESTS; i++)
    {
        uint8_t result = soil_raw_to_pct(soil_test_vectors[i].raw_value);
        soil_assert_equal(result, soil_test_vectors[i].expected_pct, soil_test_vectors[i].description);
    }
    
    LOG("\r\n========== SOIL SENSOR TEST SUMMARY ==========\r\n");
    LOG("Total: %d, Passed: %d, Failed: %d\r\n", soil_test_count, soil_test_passed, soil_test_count - soil_test_passed);
    if (soil_test_count > 0)
    {
        LOG("Pass Rate: %d%%\r\n\r\n", (soil_test_passed * 100) / soil_test_count);
    }
    
    return (soil_test_count == soil_test_passed) ? 0 : 1;
}
