#include "APDB.h"

#include "pid.h"
#include "launch_control.h"


bool launch_control_pid_struct_initialized = FALSE;

struct pid_info launch_control_pid_info_struct;


ubyte2 get_launch_control_torque_limit(float4 avg_front_wheel_speed, float4 avg_rear_wheel_speed) {
    // initialize struct if necessary
    if (!launch_control_pid_struct_initialized) {
        initialize_pid_info_struct(&launch_control_pid_info_struct,
                                    (float4) LAUNCH_CONTROL_KP,
                                    (float4) LAUNCH_CONTROL_KI,
                                    (float4) LAUNCH_CONTROL_KD,
                                    (float4) LAUNCH_CONTROL_TARGET_SLIP_RATIO);
        launch_control_pid_struct_initialized = TRUE;
    }

    // avoid divide by 0 error
    float4 wheel_slip;
    if (avg_front_wheel_speed == 0) {
        wheel_slip = (avg_rear_wheel_speed) / 0.001;
    } else {
        wheel_slip = (avg_rear_wheel_speed) / (avg_front_wheel_speed);
    }

    float4 returned_torque_limit = get_pid_output(&launch_control_pid_info_struct, wheel_slip);

    return ((ubyte2) returned_torque_limit);

}


