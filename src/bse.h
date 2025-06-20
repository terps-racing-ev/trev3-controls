#ifndef BSE_H_INCLUDED
#define BSE_H_INCLUDED

#include "IO_Driver.h"
#include "IO_CAN.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"


/**************************************************************************
 * Sensor Pins
 ***************************************************************************/
/* BSE -> pin 137 (aka adc 5V 7)*/
#define IO_PIN_BSE IO_ADC_5V_07
#define IO_BSE_SUPPLY IO_ADC_SENSOR_SUPPLY_1


/* bse */
#define BSE_MAX_VOLTAGE 4500
#define BSE_MIN_VOLTAGE 500
#define BSE_MAX_PSI 3000
#define BRAKES_ENGAGED_BSE_THRESHOLD 40 //PSI
#define BRAKE_PLAUSIBILITY_BRAKES_ENGAGED_BSE_THRESHOLD 100 //PSI

/* voltage range above max and below min that doesn't count as an error */
#define BSE_VOLTAGE_DEADZONE 100

#define BSE_NO_ERROR 0
#define BSE_OUT_OF_RANGE_ERROR 1

ubyte2 voltage_to_psi_bse(ubyte2 bse_voltage);
void get_bse(ubyte2 *bse_result, ubyte1 *error);


#endif // BSE_H_INCLUDED
