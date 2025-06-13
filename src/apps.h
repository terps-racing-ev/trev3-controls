#ifndef APPS_H_INCLUDED
#define APPS_H_INCLUDED

#include "IO_Driver.h"
#include "IO_CAN.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"


/**************************************************************************
 * Sensor Pins
 ***************************************************************************/

/* APPS 1 -> pin 152 (aka adc 5V 0)*/
#define IO_PIN_APPS_1 IO_ADC_5V_00
#define IO_APPS_1_SUPPLY IO_ADC_SENSOR_SUPPLY_0

/* APPS 2 -> pin 140 (aka adc 5V 1)*/
#define IO_PIN_APPS_2 IO_ADC_5V_01
#define IO_APPS_2_SUPPLY IO_ADC_SENSOR_SUPPLY_0


/**************************************************************************
 * Settings
 ***************************************************************************/
/* min. difference between the APPS travels
 * that'll trigger the implausibility check, 10 percent of 255 */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 100 // TODO 25 FOR TECH

#define IMPLAUSIBILITY_PERSISTENCE_PERIOD_US MsToUs(100ul)

//******************************************* 25 percent of 256
#define APPS_THRESHHOLD_BRAKE_PLAUSIBILITY 64
//*******************************************************

#define APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY 12 // 5% of 256

/* apps 1 */
#define APPS_1_MAX_VOLTAGE 4400
#define APPS_1_MIN_VOLTAGE 1039
#define APPS_1_VOLTAGE_RANGE (APPS_1_MAX_VOLTAGE - APPS_1_MIN_VOLTAGE)

/* apps 2 (with diode) */
#define APPS_2_MAX_VOLTAGE 4016
#define APPS_2_MIN_VOLTAGE 660
#define APPS_2_VOLTAGE_RANGE (APPS_2_MAX_VOLTAGE - APPS_2_MIN_VOLTAGE)

/* voltage range above max and below min that doesn't count as an error */
#define APPS_VOLTAGE_DEADZONE 250

// max number of errors in a row before torque is cut.
#define APPS_REPEATED_ERROR_MAX 20

// APPS percent travel deadzone below 0%
#define APPS_DEADZONE 2

#define APPS_NO_ERROR 0
#define APPS_1_OUT_OF_RANGE_ERROR      0x01  // binary 00000001
#define APPS_2_OUT_OF_RANGE_ERROR      0x02  // binary 00000010
#define APPS_IMPLAUSIBILITY_ERROR      0x04  // binary 00000100

ubyte1 voltage_to_pct_travel_apps_1(ubyte2 apps_1_voltage);
ubyte1 voltage_to_pct_travel_apps_2(ubyte2 apps_2_voltage);
void get_apps(ubyte1 *apps_pct_result, ubyte1 *error, ubyte1 *num_errors);

ubyte2 get_filtered_apps1_voltage(void);
ubyte2 get_filtered_apps2_voltage(void);

#endif // APPS_H_INCLUDED
