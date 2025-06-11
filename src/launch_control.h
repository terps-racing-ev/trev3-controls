#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#define LAUNCH_CONTROL_KP 1.0
#define LAUNCH_CONTROL_KI 1.0
#define LAUNCH_CONTROL_KD 0.0

#define LAUNCH_CONTROL_TARGET_SLIP_RATIO 1.0

#include "APDB.h"

ubyte2 get_launch_control_torque_limit(float4 avg_front_wheel_speed, float4 avg_rear_wheel_speed);


#endif