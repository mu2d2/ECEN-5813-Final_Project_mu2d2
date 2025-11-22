/**
 ******************************************************************************
 * @file           : state_machine.c
 * @author         : Muthuu SVS
 * @brief          : state_machine switching logic, #defines, etc
 ******************************************************************************
 */

#include "state_machine.h"

#define TRANSITION_INTERVAL (500)//500ms for updating LED brightness
#define TRANSITION_INT_TICKS (TICKMS_CONV(TRANSITION_INTERVAL))// how many systicks interrupts for interval time
#define TRANSITION_TIME (4000)//4s spent in transition time
#define TRANSITION_TICKS (TICKMS_CONV(TRANSITION_TIME))// how many systicks interrupts for transition time time

#if DEBUG
	#define STOP_TIME (2500)//2.5s spent at station
	#define STOP_TICKS (TICKMS_CONV(STOP_TIME))// how many systicks interrupts for bus stop time
#else
	#define STOP_TIME (5000)//5s spent at station
	#define STOP_TICKS (TICKMS_CONV(STOP_TIME))// how many systicks interrupts for bus stop time
#endif

#define E_STOP_TIME (5000)//5seconds mandatory at estop
#define E_STOP_TICKS (TICKMS_CONV(E_STOP_TIME))//how many systick interrupts for estop time

//LED BLINK TIMINGS
#define BLINK_TIME (100)//100ms for LED ON/OFF
#define BLINK_TICKS (TICKMS_CONV(BLINK_TIME))//how many sysTicks interrupts for blink time
#define PAUSE_TIME (400)//400ms for PAUSE LED OFF
#define PAUSE_TICKS (TICKMS_CONV(PAUSE_TIME))//how many sysTicks interrupts for pause TIME
#define BLINKS_PER_SEQ (6)//how many blinks in a sequence including on and off

//PWM Brightness Levels at each stop
#define BOULDER_BRIGHTNESS (0xFF)
#define LAFAYETTE_BRIGHTNESS (0x87)
#define DENVER_BRIGHTNESS (0x0F)

//BUS STATEMACHINE VARIABLES
static BusState ucBusLine = BOULDER_STATE;//creating Bus State Machine
static BusState prevState = BOULDER_STATE;//previous state
static BusState nextState = BOULDER_STATE;//next state
static BusState returnState = BOULDER_STATE;//return state from estop
static BusState nextStopState = BOULDER_STATE;//next stop state after transition
static uint8_t resumeFromEstop = OFF;//flag to skip timer reset when resuming from e-stop

//BLINK STATEMACHINE VARIABLES
static ledStatus eledStatus;
static ledStatus uledStatus;

//BUTTON VARIABLES
static uint8_t button_held = OFF;//if button is still held

//TIMER VARIABLES
static ticktime_t elapsed = OFF;//variable to update from timer_start
static ticktime_t temp_elapsed = OFF;//temp variable to store remaining time when entering e-stop

/*
 * initializes led status structs for ELED and ULED
 * edits file scoped led static variables
 * @param none 
 * @return none
 * Reference : 
 */
void init_led_status(void)
{
    //default starting state is OFF in boulder
    eledStatus.prevLedState = LED_STATE_OFF;
    eledStatus.thestate = LED_STATE_OFF;
    eledStatus.identity = ELED_IDENTITY;
    eledStatus.led_brightness = BOULDER_BRIGHTNESS;
    eledStatus.target_brightness = BOULDER_BRIGHTNESS;
    eledStatus.blinker_count = OFF;
    eledStatus.led_toggle = ON;
    eledStatus.pauseFlag = OFF;

    //default starting state is OFF
    uledStatus.prevLedState = LED_STATE_OFF;
    uledStatus.thestate = LED_STATE_OFF;
    uledStatus.identity = ULED_IDENTITY;
    uledStatus.led_brightness = BOULDER_BRIGHTNESS;
    uledStatus.target_brightness = BOULDER_BRIGHTNESS;
    eledStatus.blinker_count = OFF;
    uledStatus.led_toggle = ON;
    uledStatus.pauseFlag = OFF;
}

/*
 * updates State Machine of where the Bus is
 * uses function scoped BusState variables ucBusLine and nextState
 * @param none 
 * @return none
 * Reference : 
 */
void updateBusState(void)
{
    if(nextState != ucBusLine)//only update if there is a state change
    {
        ucBusLine = nextState;
    }
}

/*
 * on first entry to led state reset led timer
 * @param ledStatus *xled pointer to the led to check for first entry led states
 * @return none
 * Reference : 
 */
void first_entry_led(ledStatus *xled)
{
    if(xled->prevLedState != xled->thestate)
    {
        switch(xled->thestate)
        {
            case LED_STATE_OFF:
                break;
            case LED_BLINK:
                break;
            case LED_FADE:
                break;
            case LED_HOLD:
                break;
            default:
                break;
        }
        if(xled->identity == ELED_IDENTITY)
        {
        	reset_timer(TIMER_START_ELED_ID);//resets the blink timer
        }
        else if(xled->identity == ULED_IDENTITY)
        {
        	reset_timer(TIMER_START_ULED_ID);//resets the blink timer
        }
        else
        {
            LOG("ERROR: LED IDENTITY NOT SET PROPERLY\r\n");
        }
        xled->prevLedState = xled->thestate;//update prevled state
    }
}

/*
 * On first entry of the state prints DEBUG message, and resets timer for state and updates prev state
 * inital entry of bus stop state, sets brightness, resets led timers on first entry to state
 * uses file scoped bus State variables
 * @param none 
 * @return none
 * Reference : 
 */
void first_entry(void)
{
    //bus state first entry 
    if(prevState != ucBusLine)//if first entry into state
    {
        switch(ucBusLine)
        {
            case BOULDER_STATE:
                LOG("BUS AT BOULDER\r\n");
                eledStatus.led_brightness = BOULDER_BRIGHTNESS;
                break;
            case LAFAYETTE_STATE:
                LOG("BUS AT LAFAYETTE\r\n");
                eledStatus.led_brightness = LAFAYETTE_BRIGHTNESS;
                break;
            case DENVER_STATE:
                LOG("BUS AT DENVER\r\n");
                eledStatus.led_brightness = DENVER_BRIGHTNESS;
                break;
            case TRANS_STATE:
                LOG("BUS IN TRANSITION\r\n");
                if(prevState != E_STOP_STATE)
                {
                	eledStatus.led_brightness = eledStatus.prev_brightness;//starts fade from last stable brightness
                }//if in estop, retain led brightness
                //set target brightness for nextStop
                switch(nextStopState)
                {
                    case BOULDER_STATE:
                        eledStatus.target_brightness = BOULDER_BRIGHTNESS;
                        break;
                    case LAFAYETTE_STATE:
                        eledStatus.target_brightness = LAFAYETTE_BRIGHTNESS;
                        break;
                    case DENVER_STATE:
                        eledStatus.target_brightness = DENVER_BRIGHTNESS;
                        break;
                    default:
                        break;
                }
                if(prevState != E_STOP_STATE)//doesnt recalculate when returning from estop
				{
                	eledStatus.fade_step = ((int16_t)eledStatus.target_brightness - (int16_t)eledStatus.led_brightness)/ ((int16_t)(TRANSITION_TIME/TRANSITION_INTERVAL));//calculates fade step
				}

                break;
            case E_STOP_STATE:
                LOG("BUS E-STOP\r\n");
                uledStatus.led_brightness = BOULDER_BRIGHTNESS;
                uledStatus.prevLedState = LED_STATE_OFF;//forces first entry led logic
                break;
            default:
                break;
        }
        if(resumeFromEstop)
        {
        	restoreProgress(temp_elapsed);//restores remaining time
        	resumeFromEstop = OFF;//clears resume flag
        }
        else
        {
        	reset_timer(TIMER_START_ID);//resets timer_start to first entry
        }

        prevState = ucBusLine;//updates prevstate
    }

    //led first entry check
    first_entry_led(&eledStatus);
    first_entry_led(&uledStatus);

}

/* Executing State Machine Logic, setting brightness of LED
 * blinking status state for ELED and ULED,
 * and starting timers for each state
 * also setting next state & stop state for updating 
 * uses filed scoped bus state variables
 * @param none 
 * @return none
 * Reference : 
 */
void state_logic(void)
{
    elapsed = get_timer(TIMER_START_ID);//gets elasped time
    switch(ucBusLine)
    {
        case BOULDER_STATE:
            eledStatus.thestate = LED_BLINK;//sets eled to blinking seq loop
            if(elapsed >= STOP_TICKS)//if timer over
            {
                LOG("BOULDER -> LAFAYETTE\r\n");
                eledStatus.prev_brightness = BOULDER_BRIGHTNESS;//stores last stable brightness whether in blink or pause mode
                nextStopState = LAFAYETTE_STATE; //next bus stop
                nextState = TRANS_STATE;//send to transition
            }
            break;
        case LAFAYETTE_STATE:
            eledStatus.thestate = LED_BLINK;//sets eled to blinking seq loop
            if(elapsed >= STOP_TICKS)//if timer over
            {
                LOG("LAFAYETTE -> DENVER\r\n");
                eledStatus.prev_brightness = LAFAYETTE_BRIGHTNESS;//stores last stable brightness whether in blink or pause mode
                nextStopState = DENVER_STATE; //next bus stop
                nextState = TRANS_STATE;//send to transition
            }
            break;
        case DENVER_STATE:
            eledStatus.thestate = LED_BLINK;//sets eled to blinking seq loop
            if(elapsed >= STOP_TICKS)//if timer over
            {
                LOG("DENVER -> BOULDER\r\n");
                eledStatus.prev_brightness = DENVER_BRIGHTNESS;//stores last stable brightness whether in blink or pause mode
                nextStopState = BOULDER_STATE; //next bus stop
                nextState = TRANS_STATE;//send to transition
            }
            break;
        case TRANS_STATE:
            eledStatus.thestate = LED_FADE;//sets eled to fade logic
            if(elapsed >= TRANSITION_TICKS)//checks if reaches 4s timer
            {
                nextState = nextStopState;
            }
            break;
        case E_STOP_STATE:
            if(eledStatus.thestate == LED_FADE)//if bus is in transition hold brightness
            {
                eledStatus.thestate = LED_HOLD;
            }
            if(prevState != E_STOP_STATE)//forces first entry logic for uled
            {
            	uledStatus.prevLedState = LED_STATE_OFF;//forces first entry led logic
                reset_timer(TIMER_START_ULED_ID);
            }
            uledStatus.thestate = LED_BLINK;//start ULED blinking
            if ((elapsed >= E_STOP_TICKS) && (!get_button_state()))//if 5second timer reached and button not pressed
            {
                LOG("RESUMING ROUTE\r\n");
                nextState = returnState;
                uledStatus.thestate = LED_STATE_OFF;//turn off uled blinkings
                resumeFromEstop = ON;//set flag that we are returning from estop
            }
            break;
        default:
            break;
    }

    //updates led status and hardware
    led_handler(ucBusLine, &eledStatus);
    led_handler(ucBusLine, &uledStatus);
}

/*E_STOP HANDLER
 *sends State Machine to E-STOP from checking button state
 *resets e stop timer if repressed before 5 seconds is over
 *uses file scoped bus State variables
 * @param none 
 * @return none
 * Reference : 
 */
void estop_handler(void)
{
    if(get_button_state())//if e-stop pressed & not in E_STOP already
    {
        if(ucBusLine != E_STOP_STATE)
        {
        	temp_elapsed = get_timer(TIMER_START_ID);//gets time accured for state
            returnState = ucBusLine;//store return state
            ucBusLine = E_STOP_STATE;//set to E_STOP state
            nextState = E_STOP_STATE;//update next state to estop

            //reset uled timer only on first press 
            reset_timer(TIMER_START_ULED_ID);//resets the uled blink timer
        }
        if(button_held == OFF && (ucBusLine == E_STOP_STATE))//if estop button pressed again after being let go before timer goes off
        {
            reset_timer(TIMER_START_ID);//resets stop timer
        }
        button_held = ON;//button is now held
    }
    else//button not pressed
    {
        button_held = OFF;//turns off button held
    }
}

/*LED handler
 * timing for LED blinking, pausing, and fading logic, access led timers depending on state and LED
 * @param BusState bus for determining brightness fading, ledStatus *led to determine which led to change
 * @return none
 * Reference : 
*/
void led_handler(BusState bus, ledStatus *led)
{
	BusState tempBus = bus;//to check if in ESTOP mode
	BusState switchState = bus;//bustate used for switch logic

    ticktime_t blink_timer_local;
    if(led->identity == ELED_IDENTITY)
    {
    	blink_timer_local = get_timer(TIMER_START_ELED_ID);//gets elasped time to check blink timings
    }
    else if(led->identity == ULED_IDENTITY)
    {
    	blink_timer_local = get_timer(TIMER_START_ULED_ID);//gets elasped time to check blink timings
    }
    else
    {
        LOG("ERROR: LED IDENTITY NOT SET PROPERLY\r\n");
        blink_timer_local = OFF;
    }
    switch(led->thestate)
    {
        case LED_BLINK:
            if( !(led->pauseFlag) && (blink_timer_local >= BLINK_TICKS))//BLINK ON/OFF timer over
            {
                led->led_toggle = -(led->led_toggle);//toggle led
                if(led->led_toggle == ON)
                {
                    //keep on at current brightness level
                	if(tempBus == E_STOP_STATE)
                	{
                		switchState = returnState;//keep brightness at return state levels
                	}//otherwise do nothing 
                	switch(switchState)
                	{
                	case BOULDER_STATE:
                		led->led_brightness = BOULDER_BRIGHTNESS;
                		break;
                    case LAFAYETTE_STATE:
                		led->led_brightness = LAFAYETTE_BRIGHTNESS;
                		break;
                    case DENVER_STATE:
                		led->led_brightness = DENVER_BRIGHTNESS;
                		break;
                    case TRANS_STATE://should only be for TRANS -> ESTOP scenario
                    	led->led_brightness = BOULDER_BRIGHTNESS;//should only be ULED using this case
                	default:
                		break;
                	}
                }
                else
                {
                	led->led_brightness = OFF;//turn off
                }

                if(led->identity == ELED_IDENTITY)
                {
                	reset_timer(TIMER_START_ELED_ID);//resets the blink timer
                }
                else if(led->identity == ULED_IDENTITY)
                {
                	reset_timer(TIMER_START_ULED_ID);//resets the blink timer
                }
                else
                {
                    LOG("ERROR: LED IDENTITY NOT SET PROPERLY\r\n");
                }

                led->blinker_count++;//increase blink count

                if(led->blinker_count >= BLINKS_PER_SEQ)//3 blinks
                {
                	led->led_brightness = OFF;//turn off led for pause
                    led->pauseFlag = ON;//turn on pause flag to check for pause time
                    led->blinker_count = OFF;//reset blinker count
                    
                    //reset for pause timer
                    if(led->identity == ELED_IDENTITY)
                    {
                        reset_timer(TIMER_START_ELED_ID);//resets the pause timer
                    }
                    else if(led->identity == ULED_IDENTITY)
                    {
                        reset_timer(TIMER_START_ULED_ID);//resets the pause timer
                    }
                    else
                    {
                        LOG("ERROR: LED IDENTITY NOT SET PROPERLY\r\n");
                    }
                }
            }
            else if(led->pauseFlag && (blink_timer_local >= PAUSE_TICKS))//PAUSE timer over
            {
            	led->pauseFlag = OFF;//turn off pause
            }
            break;
        case LED_FADE:
        	led->led_toggle = ON;//keep led on
        	led->pauseFlag = OFF;//keep pauseFlag off
        	led->blinker_count = OFF;//resets blinker_count if interrupted in between blinks

            if(blink_timer_local >= TRANSITION_INT_TICKS)//if fade step timer over
            {
                //cast everything to signed int to avoid overflow issues
                int16_t current = led->led_brightness;
                int16_t target = led->target_brightness;
                current += led->fade_step;//updates brightness by fade step
                
                //clamp overshoot
                if(((led->fade_step > SIGNED_ZERO) && (current > target)) ||((led->fade_step < SIGNED_ZERO) && (current < target)))
                {
                	current = target;//set to target brightness
                }
                led->led_brightness = (uint8_t)current;//cast back to uint8_t and update brightness
                if(led->identity == ELED_IDENTITY)
                {
                	reset_timer(TIMER_START_ELED_ID);//resets the fade step timer
                }
                else
                {
                    LOG("ERROR: LED IDENTITY NOT SET PROPERLY\r\n");
                }
            }
            break;
        case LED_HOLD:
            break;
        case LED_STATE_OFF:
        	led->led_brightness = OFF;//keep LED off
            break;
        default:
            break;
    }
    set_brightness_scale(led->led_brightness, led->identity);//update led hardware
}
