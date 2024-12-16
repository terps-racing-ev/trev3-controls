#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

#include "tsil.h"
#include "utilities.h"

enum TSIL_State current_state = BLINKING_RED;
bool blinking_state = LIGHT_OFF;

// timer which controls the blinking of the light
ubyte4 light_timestamp;
bool timer_initialized = FALSE;

void set_light_to(enum TSIL_State s) {
    current_state = s;
}

// if current state is blinking red, check whether the light needs to turn
// on/off and do so.
// if current state is solid green, make sure green light is on
void do_lights_action() {
    if (current_state == BLINKING_RED) {
        // if the timer hasn't been intialized yet (i.e. first function call with
        // blinking red as the current state), initialize the timer
        if (timer_initialized == FALSE) {
            IO_RTC_StartTime(&light_timestamp);
            timer_initialized = TRUE;
        }

        // make sure green light is off
        IO_DO_Set( TSIL_GREEN_PIN, LIGHT_OFF );

        // blink on if we need to
        if (blinking_state == LIGHT_OFF && IO_RTC_GetTimeUS(light_timestamp) > BLINK_TIME_US) {
            // turn light on
            IO_DO_Set( TSIL_RED_PIN, LIGHT_ON );
            blinking_state = LIGHT_ON;

            // reset the timer
            IO_RTC_StartTime(&light_timestamp);
        } else if (blinking_state == LIGHT_ON && IO_RTC_GetTimeUS(light_timestamp) > BLINK_TIME_US) {
            // turn light off
            IO_DO_Set( TSIL_RED_PIN, LIGHT_OFF );
            blinking_state = LIGHT_OFF;

            // reset the timer
            IO_RTC_StartTime(&light_timestamp);
        }

    } else if (current_state == SOLID_GREEN) {
        // make sure red light is off
        IO_DO_Set( TSIL_RED_PIN, LIGHT_OFF );
        blinking_state = LIGHT_OFF;

        // turn on green light
        IO_DO_Set( TSIL_GREEN_PIN, LIGHT_ON );
    }
}
