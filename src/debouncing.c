#include "debouncing.h"


void initialize_debouncing_struct(struct debouncing_info* info, bool starting_state, ubyte2 thresh) {
    info->current_state = starting_state;

    info->switch_threshhold = thresh;

    info->successive_lows = 0;
    info->successive_highs = 0;
}

bool debounce_input(struct debouncing_info* info, bool input) {
    if (info->current_state == TRUE) {
        // check the given input
        if (input == FALSE) {
            info->successive_lows++;
        } else {
            info->successive_lows = 0;
        }

        // if >= than the threshold
        if (info->successive_lows >= info->switch_threshhold) {
            // flip state
            info->current_state = FALSE;

            // reset counters
            info->successive_lows = 0;
            info->successive_highs = 0;

            // return new state
            return FALSE;
        }

        // return current state
        return TRUE;
    } else {
        // check the given input
        if (input == TRUE) {
            info->successive_highs++;
        } else {
            info->successive_highs = 0;
        }

        // if >= than the threshold
        if (info->successive_highs >= info->switch_threshhold) {
            // flip state
            info->current_state = TRUE;

            // reset counters
            info->successive_highs = 0;
            info->successive_lows = 0;

            // return new state
            return TRUE;
        }

        // return current state
        return FALSE;
    }
}