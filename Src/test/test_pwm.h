/**
 ******************************************************************************
 * @file           : test_pwm.h
 * @author         : Muthuu SVS
 * @brief          : Unit tests for PWM/servo module
 *
 * Hardware: This file contains pure unit tests for the conversion function
 *           only and does not drive the PWM hardware. No sensor/servo
 *           wiring is required to run these tests. To verify physical
 *           servo wiring and movement, perform a separate hardware
 *           integration test that drives the PWM output while the
 *           servo is connected.
 ******************************************************************************
 */

#ifndef TEST_PWM_H
#define TEST_PWM_H

/**
 * Run all PWM/servo unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_pwm(void);

#endif /* TEST_PWM_H */
