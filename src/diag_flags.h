#ifndef DIAG_FLAGS_H_INCLUDED
#define DIAG_FLAGS_H_INCLUDED

#include "APDB.h"

struct diag_flags {
    /* State change flags 
    These are only reset upon succesfull rtd */
    ubyte1 apps_out_of_range_fault;
    ubyte1 apps_implausibility_fault;
    ubyte1 bse_out_of_range_fault;
    ubyte1 rtd_off_fault;
    ubyte1 sdc_off_fault;
    /* TSIL flags */
    ubyte1 imd_latch_fault;
    ubyte1 bms_latch_fault;
    ubyte1 orion_can_timeout_fault;
    /* 8 BITS */
};


void initialize_diag_flags(struct diag_flags* flags);
ubyte1 pack_diag_flags(struct diag_flags* flags);


#endif