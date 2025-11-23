/**
 ******************************************************************************
 * @file           : pwm.c
 * @author         : Muthuu SVS
 * @brief          : functions that manipulate the gpio, initializing registers
 *  for leds, and contains PWM initialization and interrupt handler for servo motor
 ******************************************************************************
 */
 #include "pwm.h"
 #include <stm32f091xc.h>
 #include "log.h"

#define F_TIM_CLOCK (48000000L)	// 48 MHz sys clk

//ULED is connected to PORTA pin 5
#define ULED_GPIO_PORT (GPIOA)//GPIO Port with LD2
#define ULED_PIN (5U)//ULED is PA5
#define ULED_OFF_MSK (GPIO_BSRR_BR_5) // ULED off mask
#define ULED_ON_MSK (GPIO_BSRR_BS_5) // ULED on mask

//SERVO is connected to PORTA pin 7
#define SERVO_GPIO_PORT (GPIOA)//GPIO Port with connection to SERVO
#define SERVO_PIN (7)//SERVO is PA7
#define SERVO_OFF_MSK (GPIO_BSRR_BS_7) // SERVO off mask
#define SERVO_ON_MSK (GPIO_BSRR_BR_7) // SERVO on mask

//SERVO PWM defines
#define PWM_TIMER (TIM3)
#define PWM_MODE (6)
#define SERVO_PWM_FREQUENCY (50)//50 Hz servo control
#define PWM_PRESCALER (48)// want 1MHz clk for timer
#define PWM_TIMER_FREQ (F_TIM_CLOCK / PWM_PRESCALER)// 1us timer frequency
#define SERVO_PWM_PERIOD (PWM_TIMER_FREQ / SERVO_PWM_FREQUENCY)//20000ticks aka 20ms for servo
#define CCR_STEP_SIZE (10)//increases speed of servo

//SERVO angle defines
#define SERVO_MIN_US (500) // 0 degrees pulse width
#define SERVO_MAX_US (2500) // 270 degrees pulse width
#define SERVO_NEUTRAL_POS (1500)
#define MIN_SERVO_ANGLE (0) // 0 degrees
#define MAX_SERVO_ANGLE (270) // 270 degrees

//PWM VARIABLES
volatile uint16_t current_ccr = SERVO_NEUTRAL_POS; //current position of servo
volatile uint16_t target_ccr = SERVO_NEUTRAL_POS; // target position of servo

/*
* Initializes gpio port peripheral and configures TIMER3 and channel 2 counter
* @param none
* @return none
* Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 232
*/
void init_PWM_SERVO(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable periph clock of gpioA
	MODIFY_FIELD(SERVO_GPIO_PORT->MODER, GPIO_MODER_MODER7, ESF_GPIO_MODER_ALT_FUNC);//configures PA7 aka SERVO into alt func mode 10 -> 2
	MODIFY_FIELD(SERVO_GPIO_PORT->AFR[OFF], GPIO_AFRL_AFRL7, ON);//sets to AF1 for TIM3_CH2

	// Configure TIM3 counter and prescaler
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	PWM_TIMER->PSC = PWM_PRESCALER - 1;//prescaler
	PWM_TIMER->ARR = SERVO_PWM_PERIOD - 1;//load-reload set to 20ms aka 19999
	PWM_TIMER->CR1 = 0; // clear CR1 and set to count up
	PWM_TIMER->CR1 |= TIM_CR1_ARPE;  // enable auto-reload preload

	// Configure TIM3 channel 2	
	PWM_TIMER->CCR2 = ON;				// Short on-time by default
	MODIFY_FIELD(PWM_TIMER->CCMR1, TIM_CCMR1_OC2M, PWM_MODE); // Select PWM mode
	PWM_TIMER->CCER &= ~TIM_CCER_CC2P;   // active-high polarity for PWM
	PWM_TIMER->CCMR1 |= TIM_CCMR1_OC2PE;	// Enable preload register
	PWM_TIMER->EGR |= TIM_EGR_UG;		// Generate update
	PWM_TIMER->CCER |= TIM_CCER_CC2E; 	// Enable channel output on OC2	
	PWM_TIMER->CCR2 =  SERVO_NEUTRAL_POS; //neutral position
	PWM_TIMER->BDTR |= TIM_BDTR_MOE;		// Enable main output
	
	//Configure TIM3 Interrupt
	PWM_TIMER->DIER |= TIM_DIER_UIE;//enables update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);//enables interrupt in NVIC

	PWM_TIMER->CR1 |= TIM_CR1_CEN;		// Enable timer 
}

/*TIMER3 Interrupt Service Routine
* PWM of SERVO
* @param none
* @return none
* Reference:
*/
void TIM3_IRQHandler(void)
{
	if(PWM_TIMER->SR & TIM_SR_UIF)//checks if timer triggered an event aka 50Hz
	{
		PWM_TIMER->SR &= ~TIM_SR_UIF;//resets update interrupt flag

		if(current_ccr != target_ccr)//checks if servo needs to move
        {
			//increment/decrement to the target position
			//clamps value to target_ccr
            if(current_ccr < target_ccr)
			{
				current_ccr+=CCR_STEP_SIZE;
				if(current_ccr > target_ccr)
				{
					current_ccr = target_ccr;
				}
			} 
            else if(current_ccr > target_ccr)
			{
				current_ccr-=CCR_STEP_SIZE;
				if(current_ccr < target_ccr)
				{
					current_ccr = target_ccr;
				}
			} 

            PWM_TIMER->CCR2 = current_ccr;//update compare register value
        }
	}
}

/*
 * initializes uled function
 * enabling gpio port and #bit to be an output
 * @param none using registers
 * @return none
 * 
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void init_uled(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable periph clock of gpioA
	MODIFY_FIELD(ULED_GPIO_PORT->MODER, GPIO_MODER_MODER5, ESF_GPIO_MODER_OUTPUT);//configures PA5 aka ld2 aka ULED into output mode 01 -> 1
}


/*
 * set ULED function
 * @param state ON or OFF
 * @return no return function
 * 
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_uled(uint8_t state)
{
	if(state)
	{
		ULED_GPIO_PORT->BSRR |= ULED_ON_MSK;//turns on ULED
	}
	else
	{
		ULED_GPIO_PORT->BSRR |= ULED_OFF_MSK;//turns off ULED
	}
}

/*
 * set SERVO function
 * @param state ON or OFF
 * @return no return function
 * 
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void set_servo(uint8_t state)
{
	if(state)
	{
		SERVO_GPIO_PORT->BSRR |= SERVO_ON_MSK;//turns on SERVO
	}
	else
	{
		SERVO_GPIO_PORT->BSRR |= SERVO_OFF_MSK;//turns off SERVO
	}
}

/* converts angle to ccr value for servo positioning
 * @param uint16_t angle desired angle between 0 and 270 degrees
 * @return uint16_t converted ccr value for servo positioning
 * 
 */
uint16_t servo_angle_to_ccr(uint16_t angle)
{
	//clamp values
    if(angle < MIN_SERVO_ANGLE)
	{
		angle = MIN_SERVO_ANGLE;
	}   
    if(angle > MAX_SERVO_ANGLE)
	{
		angle = MAX_SERVO_ANGLE;
	} 

	//converts to angle to ccr value range
    return SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / MAX_SERVO_ANGLE;//return converted ccr value
}


/* sets the position of the servo motors based on angle input
 * assigns the target_ccr variable to the converted angle value
 * @param uint16_t angle desired angle between 0 and 270 degrees
 * @return none
 */
void servo_set_angle(uint16_t angle)
{
	//set new position by updating ccr value
	target_ccr = servo_angle_to_ccr(angle);
}
