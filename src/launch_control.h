#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#define LAUNCH_CONTROL_KP 1.0
#define LAUNCH_CONTROL_KI 1.0
#define LAUNCH_CONTROL_KD 1.0

#define LAUNCH_CONTROL_TARGET_SLIP_RATIO 1.1

#define MINIMUM_TORQUE_LIMIT 100 // minimum torque limit for launch control, in Nm

#include "APDB.h"

ubyte2 get_launch_control_torque_limit(float4 avg_front_wheel_speed, float4 avg_rear_wheel_speed);


#endif