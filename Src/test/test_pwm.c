/**
 ******************************************************************************
 * @file           : test_pwm.c
 * @author         : Muthuu SVS
 * @brief          : Unit tests for PWM/servo module
 *                   Tests servo_angle_to_ccr() conversion and clamping logic
 *
 * Hardware: This file contains pure unit tests for the conversion function
 *           only and does not drive the PWM hardware. No sensor/servo
 *           wiring is required to run these tests. To verify physical
 *           servo wiring and movement, perform a separate hardware
 *           integration test that drives the PWM output while the
 *           servo is connected.
 ******************************************************************************
 */

#include "../pwm.h"
#include "../log.h"
#include <stdint.h>

//Test Defines
#define SERVO_MIN_US        (500)
#define SERVO_MAX_US        (2500)
#define MAX_SERVO_ANGLE     (270)
#define NUM_PWM_TESTS       (6)

//Test Vectors 
typedef struct {
    uint16_t angle;
    uint16_t expected_ccr;
    const char* description;
} pwm_test_vector_t;

static const pwm_test_vector_t pwm_test_vectors[NUM_PWM_TESTS] = {
    { 0,   500,  "Min angle 0 degrees" },
    { 90,  1166, "90 degrees" },
    { 135, 1500, "135 degrees midpoint" },
    { 180, 1833, "180 degrees" },
    { 270, 2500, "Max angle 270 degrees" },
    { 360, 2500, "Clamp above max 360 degrees" },
};

//Test Utilities 
static int pwm_test_count = 0;
static int pwm_test_passed = 0;

/**
 * Assert that two uint16 values are equal and log result
 * @param actual The actual value
 * @param expected The expected value
 * @param test_name Description of the test
 * @return none
 */
static void pwm_assert_equal_uint16(uint16_t actual, uint16_t expected, const char* test_name)
{
    pwm_test_count++;
    if (actual == expected)
    {
        pwm_test_passed++;
        LOG("PASS: %s (expected: %u, got: %u)\r\n", test_name, expected, actual);
    }
    else
    {
        LOG("FAIL: %s (expected: %u, got: %u)\r\n", test_name, expected, actual);
    }
}

/**
 * Run all PWM/servo unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_pwm(void)
{
    LOG("\r\n========== PWM/SERVO UNIT TESTS ==========\r\n\r\n");
    
    for (int i = 0; i < NUM_PWM_TESTS; i++)
    {
        uint16_t result = servo_angle_to_ccr(pwm_test_vectors[i].angle);
        pwm_assert_equal_uint16(result, pwm_test_vectors[i].expected_ccr, pwm_test_vectors[i].description);
    }
    
    LOG("\r\n========== PWM TEST SUMMARY ==========\r\n");
    LOG("Total: %d, Passed: %d, Failed: %d\r\n", pwm_test_count, pwm_test_passed, pwm_test_count - pwm_test_passed);
    if (pwm_test_count > 0)
    {
        LOG("Pass Rate: %d%%\r\n\r\n", (pwm_test_passed * 100) / pwm_test_count);
    }
    
    return (pwm_test_count == pwm_test_passed) ? 0 : 1;
}
