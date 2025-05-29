#ifndef DEBOUNCING_H_INCLUDED
#define DEBOUNCING_H_INCLUDED

#include "APDB.h"

// 20 cycles -> 100 ms
// 100 cycles -> 500 ms
#define THRESHHOLD_100_MS 20
#define THRESHHOLD_500_MS 100

struct debouncing_info {
    bool current_state;

    // number of cycles needed to switch states
    // if current_state = high and successive_lows >= switch_treshhold, flip to low
    // and vice versa
    ubyte2 switch_threshhold;

    // number of successive low inputs
    ubyte2 successive_lows;

    // number of successive high inputs
    ubyte2 successive_highs;
};

void initialize_debouncing_struct(struct debouncing_info* info, bool starting_state, ubyte2 thresh);

bool debounce_input(struct debouncing_info* info, bool input);

#endif