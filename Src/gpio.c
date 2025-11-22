/**
 ******************************************************************************
 * @file           : gpio.h
 * @author         : Muthuu SVS
 * @brief          : functions that manipulate the gpio, initializing registers
 *  for buttons, Leds, and setting buttons, and contains PWM initialization and interrupt handler
 ******************************************************************************
 */
 #include "gpio.h"

 //button is connected to PORTC pin 13
#define BUTTON_GPIO_PORT (GPIOC)//GPIO PORT with button
#define BUTTON_PIN (13)//pin number
#define BUTTON_IDR_MSK (GPIO_IDR_13)//button input mask
#define BUTTON_IDR (GPIOC->IDR)//button GPIO port IDR

//ULED is connected to PORTA pin 5
#define ULED_GPIO_PORT (GPIOA)//GPIO Port with LD2
#define ULED_PIN (5U)//ULED is PA5
#define ULED_OFF_MSK (GPIO_BSRR_BR_5) // ULED off mask
#define ULED_ON_MSK (GPIO_BSRR_BS_5) // ULED on mask

//ELED is connected to PORTA pin 7
#define ELED_GPIO_PORT (GPIOA)//GPIO Port with connection to ELED
#define ELED_PIN (7)//ELED is PA7
#define ELED_OFF_MSK (GPIO_BSRR_BS_7) // ELED off mask
#define ELED_ON_MSK (GPIO_BSRR_BR_7) // ELED on mask

//PWM defines
#define PWM_TIMER (TIM3)
#define F_TIM_CLOCK (48000000L)	// 48 MHz
#define PWM_FREQUENCY (500)
#define PWM_PRESCALER (2)
#define PWM_MAX_DUTY_VALUE ( (F_TIM_CLOCK / (PWM_FREQUENCY * PWM_PRESCALER)) - ON)
#define PWM_MODE (6)
#define MAX_BRIGHTNESS (0xFF)

//PWM VARIABLES
volatile uint16_t brightness_scale = 0;//PWM brightness level of ELED, volatile so interrupts can update value

/*
* Initializes gpio port peripheral and configures TIMER3 and channel 2 counter
* @param none
* @return none
* Reference: Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 7 Timers, Page No. 232
*/
void init_PWM_eled(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable periph clock of gpioA
	MODIFY_FIELD(ELED_GPIO_PORT->MODER, GPIO_MODER_MODER7, ESF_GPIO_MODER_ALT_FUNC);//configures PA7 aka ELED into alt func mode 10 -> 2
	MODIFY_FIELD(ELED_GPIO_PORT->AFR[OFF], GPIO_AFRL_AFRL7, ON);//sets to AF1 for TIM3_CH2

	// Configure TIM3 counter and prescaler
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	PWM_TIMER->PSC = PWM_PRESCALER - ON;//prescaler
	PWM_TIMER->ARR = PWM_MAX_DUTY_VALUE;//load-reload set to duty val
	PWM_TIMER->CR1 = 0; // count up

	// Configure TIM3 channel 2	
	PWM_TIMER->CCR2 = ON;				// Short on-time by default
	PWM_TIMER->CCER |= TIM_CCER_CC2P;	// active low polarity 
	MODIFY_FIELD(PWM_TIMER->CCMR1, TIM_CCMR1_OC2M, PWM_MODE); // Select PWM mode
	PWM_TIMER->CCMR1 |= TIM_CCMR1_OC2PE;	// Enable preload register
	PWM_TIMER->EGR |= TIM_EGR_UG;		// Generate update
	PWM_TIMER->CCER |= TIM_CCER_CC2E; 	// Enable channel output on OC2	
	PWM_TIMER->BDTR |= TIM_BDTR_MOE;		// Enable main output
	
	//Configure TIM3 Interrupt
	PWM_TIMER->DIER |= TIM_DIER_UIE;//enables update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);//enables interrupt in NVIC

	PWM_TIMER->CR1 |= TIM_CR1_CEN;		// Enable timer 
}

/*TIMER3 Interrupt Service Routine
* PWM of ELED, sets to brightness variable
* @param none
* @return none
* Reference:
*/
void TIM3_IRQHandler(void)
{
	if(PWM_TIMER->SR & TIM_SR_UIF)//checks if timer triggered an event aka 500Hz
	{
		PWM_TIMER->SR &= ~TIM_SR_UIF;//resets update interrupt flag

		uint16_t brightness = (PWM_MAX_DUTY_VALUE * brightness_scale)/MAX_BRIGHTNESS;//adjusts 0x0 - 0xFF scale to 0x0 - PWM_MAX_DUTY_VALUE scale without changing PWM configuration
		PWM_TIMER->CCR2 = brightness;//update brightness
	}
}

/*
 * initializes eled function
 * enabling gpio port and #bit to be an output
 * @param none using registers
 * @return none
 * 
 * Reference : Reference Manual RM0091 Chapter 8, Section 8.4.7
 * Embedded Systems Fundamentals with Arm® Cortex®-M based Microcontroller, Chapter: 2 General-​Purpose Input/​Output, Page No. 46
 */
void init_eled(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable periph clock of gpioA
	MODIFY_FIELD(ELED_GPIO_PORT->MODER, GPIO_MODER_MODER7, ESF_GPIO_MODER_OUTPUT);//configures PA7 aka ELED into output mode 01 -> 1
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
		ULED_GPIO_PORT->BSRR |= ULED_ON_MSK;//turns on ELED
	}
	else
	{
		ULED_GPIO_PORT->BSRR |= ULED_OFF_MSK;//turns off ELED
	}
}

