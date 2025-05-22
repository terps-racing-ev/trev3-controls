#ifndef DIAG_FLAGS_H_INCLUDED
#define DIAG_FLAGS_H_INCLUDED

#include "APDB.h"

struct diag_flags {
    /* State change flags that only update on a state change
    These are only reset upon succesfull rtd */
    ubyte1 apps_1_out_of_range_fault;
    ubyte1 apps_2_out_of_range_fault;
    ubyte1 apps_implausibility_fault;
    ubyte1 bse_out_of_range_fault;
    // Unlike live flags, these are true when the fault caused a bad state change
    ubyte1 rtd_off_fault;
    ubyte1 sdc_off_fault;
    /* TSIL flags */
    ubyte1 imd_latch_fault;
    ubyte1 bms_latch_fault;
    /* 8 BITS */
};

struct live_flags {
    /* live flags that don't latch and are updated every cycle */
    ubyte1 apps_1_out_of_range_fault;
    ubyte1 apps_2_out_of_range_fault;
    ubyte1 apps_implausibility_fault;
    ubyte1 bse_out_of_range_fault;
    ubyte1 rtd_val;
    ubyte1 sdc_val;
    ubyte1 imd_status;
    ubyte1 bms_status;
};


void initialize_diag_flags(struct diag_flags* flags);
void initialize_live_flags(struct live_flags* flags);
ubyte1 pack_diag_flags(struct diag_flags* flags);
ubyte1 pack_live_flags(struct live_flags* flags);


#endif