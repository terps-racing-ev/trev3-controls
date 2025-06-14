#include "APDB.h"

#include "launch_control.h"

// number of torque limit regions
// edit this however you want
#define NUM_TORQUE_LIMIT_REGIONS 3


// the different torques, in order from least to most rpm
// THERE MUST BE NUM_TORQUE_LIMIT_REGIONS ENTRIES IN THIS ARRAY
ubyte2 torque_limit_regions[NUM_TORQUE_LIMIT_REGIONS] = {100, 150, 220};

// the rpm threshholds at which to switch from one torque to another
// IN ORDER FROM LEAST TO MOST RPM
// e.g., right now, at <= 500 rpm we're at 100 Nm
// from 501 - 1500 rpm we're at 150 Nm
// at 1501+ rpm we're at 220
// THERE MUST BE NUM_TORQUE_LIMIT_REGIONS - 1 ENTRIES IN THIS ARRAY
ubyte2 rpm_boundaries[NUM_TORQUE_LIMIT_REGIONS - 1] = {500, 1500};


ubyte2 get_launch_control_torque_limit(ubyte2 rpm) {

    // if the rpm is greater than the last boundary, return the 
    // last torque
    if (rpm > rpm_boundaries[NUM_TORQUE_LIMIT_REGIONS - 2]) {
        return torque_limit_regions[NUM_TORQUE_LIMIT_REGIONS - 1];
    }

    ubyte1 i = 0;

    // now we're only considering rpms less than the last boundary
    // find the first rpm boundary the rpm is less than and return the
    // corresponding torque
    while (i < NUM_TORQUE_LIMIT_REGIONS - 1) {
        if (rpm <= rpm_boundaries[i]) {
            return torque_limit_regions[i];
        }

        i++;
    }

    // this should never be reached assuming the arrays were
    // structured correctly
    // but if they weren't, return the max torque
    return torque_limit_regions[NUM_TORQUE_LIMIT_REGIONS - 1];
}


