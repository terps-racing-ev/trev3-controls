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


float4 get_pid_output(struct pid_info* pid_struct, float4 input) {
    float4 error = pid_struct->target - input;

    float4 proportional_term = pid_struct->Kp * error;
    
    // constant amount of time between cycles (5 ms) so we
    // don't consider time in integral and derivative
    pid_struct->error_sum += error;
    float4 integral_term = pid_struct->Ki * pid_struct->error_sum;

    float4 derivative = error - pid_struct->last_error;
    float4 derivative_term = pid_struct->Kd * derivative;
    pid_struct->last_error = error;

    return (proportional_term + integral_term + derivative_term);

}