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


ubyte2 voltage_to_pct_travel_apps_1(ubyte2 apps_1_voltage) {
    if (apps_1_voltage < APPS_1_MIN_VOLTAGE) {
        return 0;
    }

    if (apps_1_voltage > APPS_1_MAX_VOLTAGE) {
        return 100;
    }

    return ((apps_1_voltage - APPS_1_MIN_VOLTAGE) / (APPS_1_VOLTAGE_RANGE / 100));

}

ubyte2 voltage_to_pct_travel_apps_2(ubyte2 apps_2_voltage) {
    if (apps_2_voltage < APPS_2_MIN_VOLTAGE) {
        return 0;
    }

    if (apps_2_voltage > APPS_2_MAX_VOLTAGE) {
        return 100;
    }

    return ((apps_2_voltage - APPS_2_MIN_VOLTAGE) / (APPS_2_VOLTAGE_RANGE / 100));

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



// gets apps values, puts average pct travel into apps_pct_result
// if an error is detected, turns error to TRUE
void get_apps(ubyte2 *apps_pct_result, ubyte1 *error, ubyte1 *num_errors) {
    if (!moving_average_structs_initialized) {
        initialize_moving_average_struct(&apps_1_moving_average_info);
        initialize_moving_average_struct(&apps_2_moving_average_info);
        moving_average_structs_initialized = TRUE;
    }


    ubyte2 apps_1_val;
    bool apps_1_fresh;
    ubyte2 apps_2_val;
    bool apps_2_fresh;

    // get voltage values
    IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);
    IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

    // uncomment once ready to use moving average filter
    // apps_1_val = filter_point(apps_1_val, &apps_1_moving_average_info);
    // apps_2_val = filter_point(apps_2_val, &apps_2_moving_average_info);
    apps_1_val = get_filtered_apps1_voltage();
    apps_2_val = get_filtered_apps2_voltage();

    bool apps_1_within_threshhold = (apps_1_val >= (APPS_1_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_1_val <= (APPS_1_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));
    bool apps_2_within_threshhold = (apps_2_val >= (APPS_2_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_2_val <= (APPS_2_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));

    ubyte4 implausibility_timestamp;
    ubyte2 apps_1_pct;
    ubyte2 apps_2_pct;
    sbyte2 difference;

    // check that both apps are within the treshhold
    if (apps_1_within_threshhold &&
        apps_2_within_threshhold) {

        apps_1_pct = voltage_to_pct_travel_apps_1(apps_1_val);

        if (apps_1_pct < 0) {
            apps_1_pct = 0;
        }

        apps_2_pct = voltage_to_pct_travel_apps_2(apps_2_val);

        if (apps_2_pct < 0) {
            apps_2_pct = 0;
        }

        difference = apps_1_pct - apps_2_pct;


        // check if there's an implausible deviation between the two
        if ((Abs(difference)) >= APPS_MIN_IMPLAUSIBLE_DEVIATION) {
            IO_RTC_StartTime(&implausibility_timestamp);

            // continuously check APPS for 100 ms to check if it stays implausible
            while (IO_RTC_GetTimeUS(implausibility_timestamp) <= IMPLAUSIBILITY_PERSISTENCE_PERIOD_US) {
                IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);
                IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

                // uncomment once ready to use moving average filter
                apps_1_val = filter_point(apps_1_val, &apps_1_moving_average_info);
                apps_2_val = filter_point(apps_2_val, &apps_2_moving_average_info);

                apps_1_within_threshhold = (apps_1_val >= (APPS_1_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_1_val <= (APPS_1_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));
                apps_2_within_threshhold = (apps_2_val >= (APPS_2_MIN_VOLTAGE - APPS_VOLTAGE_DEADZONE)) && (apps_2_val <= (APPS_2_MAX_VOLTAGE + APPS_VOLTAGE_DEADZONE));

                // if the values are plausible
                if (apps_1_within_threshhold && apps_2_within_threshhold) {
                    apps_1_pct = voltage_to_pct_travel_apps_1(apps_1_val);
                    apps_2_pct = voltage_to_pct_travel_apps_2(apps_2_val);

                    difference = apps_1_pct - apps_2_pct;

                    // then return
                    if ((Abs(difference)) < APPS_MIN_IMPLAUSIBLE_DEVIATION) {
                        *apps_pct_result = ((apps_1_pct) + (apps_2_pct)) / 2;

                        if (*apps_pct_result <= APPS_DEADZONE) {
                            *apps_pct_result = 0;
                        }
                        *error = APPS_NO_ERROR;
                        *num_errors = 0;
                        return;
                    }
                }
            }

            // if the values never became plausible, then indicate an error
            *apps_pct_result = 0;

            // only error out if APPS_REPEATED_ERROR_MAX errors have been encountered in a row
            (*num_errors)++;
            if ((*num_errors) > APPS_REPEATED_ERROR_MAX && !(IGNORE_APPS_ERROR)) {
                *error = APPS_IMPLAUSIBILITY_ERROR;
            }
            return;
        }


        // set the result to the average of the two
        *apps_pct_result = ((apps_1_pct) + (apps_2_pct)) / 2;
        if (*apps_pct_result <= APPS_DEADZONE) {
            *apps_pct_result = 0;
        }
        *num_errors = 0;
        *error = APPS_NO_ERROR;
    } else {
        // only error out if APPS_REPEATED_ERROR_MAX errors have been encountered in a row
        (*num_errors)++;
        if ((*num_errors) > APPS_REPEATED_ERROR_MAX && !(IGNORE_APPS_ERROR)) {
            *error = APPS_OUT_OF_RANGE_ERROR;
        }
        *apps_pct_result = 0;
    }
}
