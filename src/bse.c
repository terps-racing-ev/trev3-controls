#include "IO_Driver.h"
#include "IO_CAN.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

#include "moving_average.h"
#include "bse.h"
#include "debug_defines.h"

bool moving_average_struct_initialized = FALSE;
struct moving_average_info bse_moving_average_info;

void get_bse(ubyte2 *bse_result, ubyte1 *error) {
    if (!moving_average_struct_initialized) {
        initialize_moving_average_struct(&bse_moving_average_info);
        moving_average_struct_initialized = TRUE;
    }


    ubyte2 bse_val;
    bool bse_fresh;

    // get voltage from pin
    IO_ADC_Get(IO_PIN_BSE, &bse_val, &bse_fresh);

    // uncomment to use moving average filter
    // bse_val = filter_point(bse_val, &bse_moving_average_info);

    // check if its in the treshhold
    bool bse_within_threshhold = (bse_val >= BSE_MIN_VOLTAGE) && (bse_val <= BSE_MAX_VOLTAGE);

    //error out if it isn't
    if (bse_within_threshhold) {
        *bse_result = bse_val;
        *error = BSE_NO_ERROR;
    } else {
        *bse_result = 0;
        *error = IGNORE_BSE_ERROR ? BSE_NO_ERROR : BSE_OUT_OF_RANGE_ERROR;
    }
}
