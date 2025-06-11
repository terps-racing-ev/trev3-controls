#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

#include "APDB.h"

struct pid_info {
    float4 Kp;
    float4 Ki;
    float4 Kd;

    float4 target;

    // for integral
    float4 error_sum;

    // for derivative
    float4 last_error;

    ubyte4 last_error_timestamp;
    bool timestamp_initialized;
};

void initialize_pid_info_struct(struct pid_info* pid_struct,
                                float4 Kp,
                                float4 Ki,
                                float4 Kd,
                                float4 target);


float4 get_pid_output(struct pid_info* pid_struct, float4 input);


#endif