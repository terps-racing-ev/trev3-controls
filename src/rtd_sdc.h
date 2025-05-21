#ifndef RTD_SDC_H_INCLUDED
#define RTD_SDC_H_INCLUDED

/**************************************************************************
* RTD Settings
***************************************************************************/
#define RTD_ON 0
#define RTD_OFF 1

// number of successive cycles for debouncing is 100 ms
#define RTD_DEBOUNCING_THRESH THRESHHOLD_100_MS

/**************************************************************************
* SDC Settings
***************************************************************************/
#define SDC_ON 1
#define SDC_OFF 0

// number of successive cycles for debouncing is 100 ms
#define SDC_DEBOUNCING_THRESH THRESHHOLD_100_MS

/**************************************************************************
 * Sensor Pins
 ***************************************************************************/
/* RTD -> pin 263 (aka digital in 0) (switched to ground with pullup) */
#define IO_PIN_RTD IO_DI_00

/* SDC -> pin 256 (aka digital in 1) */
#define IO_PIN_SDC IO_DI_01


void get_rtd(bool *rtd_val);
void get_sdc(bool *sdc_val);

#endif