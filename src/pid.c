#include "APDB.h"
#include "pid.h"
#include "IO_RTC.h"

void initialize_pid_info_struct(struct pid_info* pid_struct,
                                float4 Kp,
                                float4 Ki,
                                float4 Kd,
                                float4 target) {


    pid_struct->Kp = Kp;
    pid_struct->Ki = Ki;
    pid_struct->Kd = Kd;

    pid_struct->target = target;

    pid_struct->error_sum = 0;
    pid_struct->last_error = 0;

    pid_struct->timestamp_initialized = FALSE;
    pid_struct->last_error_timestamp = 0;
}


float4 get_pid_output(struct pid_info* pid_struct, float4 input) {
    float4 error = pid_struct->target - input;

    float4 proportional_term = pid_struct->Kp * error;

    float4 integral_term;
    float4 derivative_term;
    if (pid_struct->timestamp_initialized) {
        ubyte4 time_since_last_error = IO_RTC_GetTimeUS(pid_struct->last_error_timestamp);

        pid_struct->error_sum += (error * ((float4) time_since_last_error));
        integral_term = pid_struct->Ki * (pid_struct->error_sum);
        
        derivative_term = pid_struct->Kd * ((error - pid_struct->last_error) / ((float4) time_since_last_error));

        pid_struct->last_error = error;

        IO_RTC_StartTime(&pid_struct->last_error_timestamp);
    } else {
        IO_RTC_StartTime(&pid_struct->last_error_timestamp);

        pid_struct->timestamp_initialized = TRUE;

        pid_struct->error_sum += error;
        pid_struct->last_error = error;

        integral_term = 0;
        derivative_term = 0;
    }

    return (proportional_term + integral_term + derivative_term);

}