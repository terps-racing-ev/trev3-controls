#include "IO_Driver.h"
#include "IO_CAN.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

#include "apps.h"
#include "utilities.h"
#include "moving_average.h"
#include "debug_defines.h"


bool moving_average_structs_initialized = FALSE;

struct moving_average_info apps_1_moving_average_info;
struct moving_average_info apps_2_moving_average_info;


ubyte1 voltage_to_pedal_travel_apps_1(ubyte2 apps_1_voltage) {
    if (apps_1_voltage < APPS_1_MIN_VOLTAGE) {
        return 0;
    }

    if (apps_1_voltage > APPS_1_MAX_VOLTAGE) {
        return 255;
    }

    return (ubyte1)(((ubyte4)(apps_1_voltage - APPS_1_MIN_VOLTAGE) * 255) / APPS_1_VOLTAGE_RANGE);

}

ubyte1 voltage_to_pedal_travel_apps_2(ubyte2 apps_2_voltage) {
    if (apps_2_voltage < APPS_2_MIN_VOLTAGE) {
        return 0;
    }

    if (apps_2_voltage > APPS_2_MAX_VOLTAGE) {
        return 255;
    }

    return (ubyte1)(((ubyte4)(apps_2_voltage - APPS_2_MIN_VOLTAGE) * 255) / APPS_2_VOLTAGE_RANGE);

}

ubyte2 get_filtered_apps1_voltage(void) {
    if (moving_average_structs_initialized) {
        ubyte2 apps_1_val;
        bool apps_1_fresh;

        // get voltage values
        IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);

        // uncomment once ready to use moving average filter
        return filter_point(apps_1_val, &apps_1_moving_average_info);
        // return apps_1_val;
    }

    return 0;
}

ubyte2 get_filtered_apps2_voltage(void) {
    if (moving_average_structs_initialized) {
        ubyte2 apps_2_val;
        bool apps_2_fresh;

        // get voltage values
        IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

        // uncomment once ready to use moving average filter
        return filter_point(apps_2_val, &apps_2_moving_average_info);
        // return apps_2_val;
    }

    return 0;
}

// gets apps values, puts average pct travel into apps_pedal_travel_result
// if an error is detected, sets error to error code if num_errors is greater than APPS_REPEATED_ERROR_MAX
void get_apps(ubyte1 *apps_pedal_travel_result, ubyte1 *error, ubyte1 *num_errors) {
    if (!moving_average_structs_initialized) {
        initialize_moving_average_struct(&apps_1_moving_average_info);
        initialize_moving_average_struct(&apps_2_moving_average_info);
        moving_average_structs_initialized = TRUE;
    }

    // Reset error every call for bit masking
    *error = APPS_NO_ERROR;

    ubyte2 apps_1_val = get_filtered_apps1_voltage();;
    ubyte2 apps_2_val = get_filtered_apps2_voltage();; 

    bool apps_1_within_threshhold = (apps_1_val >= (APPS_1_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_1_val <= (APPS_1_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));
    bool apps_2_within_threshhold = (apps_2_val >= (APPS_2_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_2_val <= (APPS_2_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));

    ubyte1 apps_1_pedal_travel;
    ubyte1 apps_2_pedal_travel;
    sbyte2 difference;

    // check that both apps are within the treshhold
    if (apps_1_within_threshhold &&
        apps_2_within_threshhold) {

        apps_1_pedal_travel = voltage_to_pedal_travel_apps_1(apps_1_val);
        apps_2_pedal_travel = voltage_to_pedal_travel_apps_2(apps_2_val);

        difference = apps_1_pedal_travel - apps_2_pedal_travel;

        // check if there's an implausible deviation between the two
        if ((Abs(difference)) >= APPS_MIN_IMPLAUSIBLE_DEVIATION) {
            // only error out if APPS_REPEATED_ERROR_MAX errors have been encountered in a row
            
            if ((*num_errors) > APPS_REPEATED_ERROR_MAX && !(IGNORE_APPS_ERROR)) {
                *error |= APPS_IMPLAUSIBILITY_ERROR;
            } else {
                (*num_errors)++;
            }
            *apps_pedal_travel_result = 0;
            return;
        }

        // set the result to the average of the two
        *apps_pedal_travel_result = ((apps_1_pedal_travel) + (apps_2_pedal_travel)) / 2;
        if (*apps_pedal_travel_result <= APPS_DEADZONE) {
            *apps_pedal_travel_result = 0;
        }
        *num_errors = 0;
        
    } else {
        // only error out if APPS_REPEATED_ERROR_MAX errors have been encountered in a row       
        if ((*num_errors) > APPS_REPEATED_ERROR_MAX && !(IGNORE_APPS_ERROR)) {
            if (!apps_1_within_threshhold) {
                *error |= APPS_1_OUT_OF_RANGE_ERROR;
            }
            if (!apps_2_within_threshhold) {
                *error |= APPS_2_OUT_OF_RANGE_ERROR;
            }
        } else {
            (*num_errors)++;
        }
        *apps_pedal_travel_result = 0;
    }
}
