#ifndef BSE_H_INCLUDED
#define BSE_H_INCLUDED

#include "APDB.h"
#include "IO_ADC.h"
#include "IO_CAN.h"
#include "IO_DIO.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

/**************************************************************************
 * Sensor Pins
 ***************************************************************************/
/* BSE -> pin 147 (aka adc 5V 7)*/
#define IO_PIN_BSE IO_ADC_5V_07
#define IO_BSE_SUPPLY IO_ADC_SENSOR_SUPPLY_1

// Regen constants
// TODO: Update regen constants during brake tuning
#define FRONT_BRAKE_SLOPE 0
#define FRONT_BRAKE_OFFSET 0

#define REAR_BRAKE_SLOPE 0
#define REAR_BRAKE_OFFSET 0

// Regen functions
ubyte2 get_regen_torque(ubyte2 bse_voltage);

// BSE constants
#define BSE_MAX_VOLTAGE 4510
#define BSE_MIN_VOLTAGE 100
#define BRAKES_ENGAGED_BSE_THRESHOLD 550
#define BRAKE_PLAUSIBILITY_BRAKES_ENGAGED_BSE_THRESHOLD 1800

// BSE functions
void get_bse(ubyte2 *bse_result, bool *error);

#endif  // BSE_H_INCLUDED
