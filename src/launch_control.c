#include "APDB.h"

#include "pid.h"
#include "launch_control.h"


bool launch_control_pid_struct_initialized = FALSE;

struct pid_info launch_control_pid_info_struct;


ubyte2 get_launch_control_torque_limit(float4 torque, float4 wheel_slip) {
    // initialize struct if necessary
    if (!launch_control_pid_struct_initialized) {
        initialize_pid_info_struct(&launch_control_pid_info_struct,
                                    (float4) LAUNCH_CONTROL_KP,
                                    (float4) LAUNCH_CONTROL_KI,
                                    (float4) LAUNCH_CONTROL_KD,
                                    (float4) LAUNCH_CONTROL_TARGET_SLIP_RATIO);
        launch_control_pid_struct_initialized = TRUE;
    }

    // get_pid_output should return negative number when wheelspin happens
    float4 returned_torque_limit = torque + get_pid_output(&launch_control_pid_info_struct, wheel_slip);
    if (returned_torque_limit < MINIMUM_TORQUE_LIMIT) returned_torque_limit = MINIMUM_TORQUE_LIMIT;
    // not mutually exclusive
    if (returned_torque_limit > torque) returned_torque_limit = torque;
    return ((ubyte2) returned_torque_limit);

}


