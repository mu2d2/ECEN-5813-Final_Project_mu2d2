/**
 ******************************************************************************
 * @file           : test_state_machine.h
 * @author         : Muthuu SVS
 * @brief          : Unit tests for state machine module
 *
 * Hardware: These are pure unit tests for state-name and threshold
 *           constants and do not require sensors or actuators to be
 *           connected. To exercise the full FSM on hardware (sensors,
 *           valves, servo), run the application on a board with the
 *           appropriate peripherals wired and observe runtime behavior.
 ******************************************************************************
 */

#ifndef TEST_STATE_MACHINE_H
#define TEST_STATE_MACHINE_H

/**
 * Run all state machine unit tests
 * @param none
 * @return 0 if all tests pass, 1 if any fail
 */
int test_state_machine(void);

#endif /* TEST_STATE_MACHINE_H */
