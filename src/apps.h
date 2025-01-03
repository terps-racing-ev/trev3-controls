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
/* min. difference between the APPS pct travels
 * that'll trigger the implausibility check */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 10

#define IMPLAUSIBILITY_PERSISTENCE_PERIOD_US MsToUs(100ul)

//***************************************** used to be 25
#define APPS_THRESHHOLD_BRAKE_PLAUSIBILITY 25
//*******************************************************

#define APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY 5

/* apps 1 */
#define APPS_1_MAX_VOLTAGE 4550
#define APPS_1_MIN_VOLTAGE 1280
#define APPS_1_VOLTAGE_RANGE (APPS_1_MAX_VOLTAGE - APPS_1_MIN_VOLTAGE)

/* apps 2 */
#define APPS_2_MAX_VOLTAGE 3300
#define APPS_2_MIN_VOLTAGE 250
#define APPS_2_VOLTAGE_RANGE (APPS_2_MAX_VOLTAGE - APPS_2_MIN_VOLTAGE)

/* voltage range above max and below min that doesn't count as an error */
#define APPS_VOLTAGE_DEADZONE 150

// max number of errors in a row before torque is cut.
#define APPS_REPEATED_ERROR_MAX 5

// APPS percent travel deadzone below 0%
#define APPS_DEADZONE 2

#define APPS_IMPLAUSIBILITY_ERROR 1
#define APPS_OUT_OF_RANGE_ERROR 2



ubyte2 voltage_to_pct_travel_apps_1(ubyte2 apps_1_pct);
ubyte2 voltage_to_pct_travel_apps_2(ubyte2 apps_2_pct);
void get_apps(ubyte2 *apps_pct_result, bool *error, ubyte1 *num_errors);


#endif // APPS_H_INCLUDED
