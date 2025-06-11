#include "APDB.h"
#include "pid.h"

void initialize_pid_info_struct(struct pid_info* pid_struct,
                                float4 Kp,
                                float4 Ki,
                                float4 Kd,
                                float4 target) {


    pid_struct->Kp = Kp;
    pid_struct->Kp = Ki;
    pid_struct->Kp = Kd;

    pid_struct->target = target;

    pid_struct->error_sum = 0;
    pid_struct->last_error = 0;
}