#ifndef TSIL_H_INCLUDED
#define TSIL_H_INCLUDED

#include "IO_Driver.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"
#include "utilities.h"

#define TSIL_RED_PIN IO_DO_00
#define TSIL_GREEN_PIN IO_DO_01

#define LIGHT_ON TRUE
#define LIGHT_OFF FALSE

// 250 ms time on and off
#define BLINK_TIME_US MsToUs(250)

enum TSIL_State { SOLID_GREEN, BLINKING_RED };

void do_lights_action(void);
void set_light_to(enum TSIL_State s);

#endif // TSIL_H_INCLUDED