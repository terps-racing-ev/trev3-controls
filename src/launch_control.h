#ifndef LAUNCH_CONTROL_H
#define LAUNCH_CONTROL_H

#include "APDB.h"

#define LAUNCH_CONTROL_KP 10.0
#define LAUNCH_CONTROL_KI 0.0
#define LAUNCH_CONTROL_KD 0.0

#define LAUNCH_CONTROL_TARGET_SLIP_RATIO 1.1

#define MINIMUM_TORQUE_LIMIT 10 // minimum torque limit for launch control, in Nm


ubyte2 get_launch_control_torque_limit(float4 torque, float4 wheel_slip);


#endif