#include "diag_flags.h"

void initialize_diag_flags(struct diag_flags* flags) {
    flags->apps_out_of_range_fault = 0;
    flags->apps_implausibility_fault = 0;
    flags->bse_out_of_range_fault = 0;
    flags->rtd_off_fault = 0;
    flags->sdc_off_fault = 0;
    /* TSIL flags */
    flags->imd_latch_fault = 0;
    flags->bms_latch_fault = 0;
    flags->orion_can_timeout_fault = 0;
}


ubyte1 pack_diag_flags(struct diag_flags* flags) {
    ubyte1 result = 0;

    result |= (flags->apps_out_of_range_fault << 7);
    result |= (flags->apps_implausibility_fault << 6);
    result |= (flags->bse_out_of_range_fault << 5);
    result |= (flags->rtd_off_fault << 4);
    result |= (flags->sdc_off_fault << 3);
    result |= (flags->imd_latch_fault << 2);
    result |= (flags->bms_latch_fault << 1);
    result |= (flags->orion_can_timeout_fault);

    return result;
}

