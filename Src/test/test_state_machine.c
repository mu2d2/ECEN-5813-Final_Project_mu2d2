/**
 ******************************************************************************
 * @file           : test_state_machine.c
 * @author         : Muthuu SVS
 * @brief          : Unit tests for state machine module
 *                   Tests state name conversion and threshold logic
 *
 * Hardware: These are pure unit tests for state-name and threshold
 *           constants and do not require sensors or actuators to be
 *           connected. To exercise the full FSM on hardware (sensors,
 *           valves, servo), run the application on a board with the
 *           appropriate peripherals wired and observe runtime behavior.
 ******************************************************************************
 */

#include "../state_machine.h"
#include "../log.h"
#include <string.h>
#include <stdint.h>

//Test Defines
#define NUM_STATE_TESTS (12)

//Test Vectors
typedef struct {
    water_state_t state;
    const char* expected_name;
    const char* description;
} state_test_vector_t;

static const state_test_vector_t state_test_vectors[NUM_STATE_TESTS] = {
    { WATER_IDLE,         "WATER_IDLE",         "State name: WATER_IDLE" },
    { WATER_MEASURE,      "WATER_MEASURE",      "State name: WATER_MEASURE" },
    { WATER_WAIT_MEASURE, "WATER_WAIT_MEASURE", "State name: WATER_WAIT_MEASURE" },
    { WATER_DECIDE,       "WATER_DECIDE",       "State name: WATER_DECIDE" },
    { WATER_OPEN_VALVE,   "WATER_OPEN_VALVE",   "State name: WATER_OPEN_VALVE" },
    { WATER_WATERING,     "WATER_WATERING",     "State name: WATER_WATERING" },
    { WATER_CLOSE_VALVE,  "WATER_CLOSE_VALVE",  "State name: WATER_CLOSE_VALVE" },
};

typedef struct {
    int threshold_value;
    int expected_value;
    const char* description;
} threshold_test_vector_t;

static const threshold_test_vector_t threshold_test_vectors[] = {
    { DYING_OF_THIRST_THRESHOLD, 500,  "DYING_OF_THIRST_THRESHOLD" },
    { DRY_THRESHOLD,             1000, "DRY_THRESHOLD" },
    { WET_THRESHOLD,             2100, "WET_THRESHOLD" },
    { SOAKING_THRESHOLD,         2800, "SOAKING_THRESHOLD" },
};

//Test Utilities 
static int sm_test_count = 0;
static int sm_test_passed = 0;

/**
 * Assert that two strings are equal and log result
 * @param actual The actual string
 * @param expected The expected string
 * @param test_name Description of the test
 * @return none
 */
static void sm_assert_string_equal(const char* actual, const char* expected, const char* test_name)
{
    sm_test_count++;
    if (strcmp(actual, expected) == 0)
    {
        sm_test_passed++;
        LOG("PASS: %s\r\n", test_name);
    }
    else
    {
        LOG("FAIL: %s (expected: '%s', got: '%s')\r\n", test_name, expected, actual);
    }
}

/**
 * Assert that two int values are equal and log result
 * @param actual The actual value
 * @param expected The expected value
 * @param test_name Description of the test
 * @return none
 */
static void sm_assert_equal_int(int actual, int expected, const char* test_name)
{
    sm_test_count++;
    if (actual == expected)
    {
        sm_test_passed++;
        LOG("PASS: %s (expected: %d, got: %d)\r\n", test_name, expected, actual);
    }
    else
    {
        LOG("FAIL: %s (expected: %d, got: %d)\r\n", test_name, expected, actual);
    }
}

/**
 * Run all state machine unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_state_machine(void)
{
    LOG("\r\n========== STATE MACHINE UNIT TESTS ==========\r\n\r\n");
    
    LOG("--- State Name Tests ---\r\n");
    for (int i = 0; i < 7; i++)
    {
        const char* result = water_state_to_string(state_test_vectors[i].state);
        sm_assert_string_equal(result, state_test_vectors[i].expected_name, state_test_vectors[i].description);
    }
    
    /* Test invalid state */
    sm_assert_string_equal(water_state_to_string(99), "UNKNOWN_STATE", "Invalid state returns UNKNOWN_STATE");
    
    LOG("\r\n--- Threshold Tests ---\r\n");
    for (int i = 0; i < 4; i++)
    {
        sm_assert_equal_int(threshold_test_vectors[i].threshold_value, 
                           threshold_test_vectors[i].expected_value, 
                           threshold_test_vectors[i].description);
    }
    
    LOG("\r\n========== STATE MACHINE TEST SUMMARY ==========\r\n");
    LOG("Total: %d, Passed: %d, Failed: %d\r\n", sm_test_count, sm_test_passed, sm_test_count - sm_test_passed);
    if (sm_test_count > 0)
    {
        LOG("Pass Rate: %d%%\r\n\r\n", (sm_test_passed * 100) / sm_test_count);
    }
    
    return (sm_test_count == sm_test_passed) ? 0 : 1;
}
