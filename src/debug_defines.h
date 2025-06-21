#ifndef DEBUG_DEFINES_H_INCLUDED
#define DEBUG_DEFINES_H_INCLUDED

/***************************************************************************
* DEBUG VARIABLES
***************************************************************************/

// whatever you want to ignore set to TRUE

// ignore the switch when doing rtd
#define IGNORE_RTD_SWITCH FALSE
// ignore the brake pressing threshold when doing rtd
#define IGNORE_RTD_BRAKES FALSE

// ignore these error conditions
#define IGNORE_APPS_IMPLAUSIBILITY TRUE
#define IGNORE_APPS_OOR FALSE
#define IGNORE_BSE_ERROR FALSE
#define IGNORE_SDC_OFF FALSE

// turn off the brake plausibility code
#define IGNORE_BRAKE_PLAUSIBILITY FALSE

// timeout turns tsil blinking red
#define TSIL_TIMEOUT_ENABLED FALSE

// send current limiting messages to inverter
#define CURRENT_LIMITING_ENABLED TRUE


#define LAUNCH_CONTROL_ENABLED FALSE


#endif
