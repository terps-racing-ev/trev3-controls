#include "IO_Driver.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

#include "debouncing.h"
#include "debug_defines.h"
#include "rtd_sdc.h"


bool rtd_struct_initialized = FALSE;
bool sdc_struct_initialized = FALSE;

struct debouncing_info rtd_debouncing_info;
struct debouncing_info sdc_debouncing_info;

void get_rtd(bool *rtd_val) {
    if (!rtd_struct_initialized) {
        // starting value for rtd is off
        initialize_debouncing_struct(&rtd_debouncing_info, RTD_OFF, RTD_DEBOUNCING_THRESH);

        rtd_struct_initialized = TRUE;
    }

    if (IGNORE_RTD_SWITCH) {
        *rtd_val = RTD_ON;
    } else {
        bool rtd_raw;
        IO_DI_Get(IO_PIN_RTD, &rtd_raw);

        *rtd_val = debounce_input(&rtd_debouncing_info, rtd_raw);
    }    
}

void get_sdc(bool *sdc_val) {
    if (!sdc_struct_initialized) {
        // starting value for sdc is on
        initialize_debouncing_struct(&sdc_debouncing_info, SDC_ON, SDC_DEBOUNCING_THRESH);

        sdc_struct_initialized = TRUE;
    }


    if (IGNORE_SDC_OFF) {
        *sdc_val = SDC_ON;
    } else {
        bool sdc_raw;
        IO_DI_Get(IO_PIN_SDC, &sdc_raw);

        *sdc_val = debounce_input(&sdc_debouncing_info, sdc_raw);
    }
}