/**************************************************************************
 *  XC-2000 IO-LIB
 *
 *  Template
 **************************************************************************
 *
 *  Quickstart Template for HY-TTC60
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_CAN.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

/**************************************************************************
 * Utility Macros
 ***************************************************************************/

/* absolute value macro (so we don't have to import all of stdlib.h
 * for one macro)
 */
#define Abs(k) (((k) < 0) ? ((-1) * (k)) : (k))

/* macro to convert a milliseconds time to microseconds (for rtc functions) */
#define MsToUs(x) ((x)*(1000ul))


/**************************************************************************
 * Sensor Pins
 ***************************************************************************/

/* APPS 1 -> pin 152 (aka adc 5V 0)*/
#define IO_PIN_APPS_1 IO_ADC_5V_00
#define IO_APPS_1_SUPPLY IO_ADC_SENSOR_SUPPLY_0

/* APPS 2 -> pin 140 (aka adc 5V 1)*/
#define IO_PIN_APPS_2 IO_ADC_5V_01
#define IO_APPS_2_SUPPLY IO_ADC_SENSOR_SUPPLY_0

/* BSE -> pin 151 (aka adc 5V 2)*/
#define IO_PIN_BSE IO_ADC_5V_02
#define IO_BSE_SUPPLY IO_ADC_SENSOR_SUPPLY_1

/* RTD -> pin 263 (aka digital in 0) (switched to ground with pullup) */
#define IO_PIN_RTD IO_DI_00

/* SDC -> pin 256 (aka digital in 1) */
#define IO_PIN_SDC IO_DI_01

/* Buzzer */
#define IO_PIN_BUZZER IO_DO_12
/**************************************************************************
 * RTD Settings
 ***************************************************************************/
#define RTD_ON 0
#define RTD_OFF 1

// 1.5 s RTD sound time
#define RTD_SOUND_DURATION MsToUs(1500ul)

/**************************************************************************
 * APPS, BSE Settings
 ***************************************************************************/

/* min. difference between the APPS pct travels
 * that'll trigger the implausibility check */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 10

#define IMPLAUSIBILITY_PERSISTENCE_PERIOD_US MsToUs(100ul)

#define APPS_THRESHHOLD_BRAKE_PLAUSIBILITY 25

#define APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY 5

/* apps 1 */

#define APPS_1_MAX_VOLTAGE 3250

#define APPS_1_MIN_VOLTAGE 850

#define APPS_1_VOLTAGE_RANGE (APPS_1_MAX_VOLTAGE - APPS_1_MIN_VOLTAGE)

/* apps 2 */

#define APPS_2_MAX_VOLTAGE 4450

#define APPS_2_MIN_VOLTAGE 930

#define APPS_2_VOLTAGE_RANGE (APPS_2_MAX_VOLTAGE - APPS_2_MIN_VOLTAGE)


/* bse */
#define BSE_MAX_VOLTAGE 4500

#define BSE_MIN_VOLTAGE 500

#define BRAKES_ENGAGED_BSE_THRESHOLD 1000

/* percent to torque algorithm */
#define PCT_TRAVEL_TO_TORQUE(val) ((val) * 2)
/**************************************************************************
 * SDC Settings
 ***************************************************************************/
#define SDC_ON 1
#define SDC_OFF 0

/**************************************************************************
 * Controls Math Macros
 ***************************************************************************/

/* TODO: voltage->pct travel code*/
// converts voltage into percent from 0 - 100
#define VoltageToPctTravelApps1(val) ((((val) - APPS_1_MIN_VOLTAGE) / (APPS_1_VOLTAGE_RANGE / 100)))
#define VoltageToPctTravelApps2(val) ((((val) - APPS_2_MIN_VOLTAGE) / (APPS_2_VOLTAGE_RANGE / 100)))


/**************************************************************************
 * CAN Constants
 ***************************************************************************/

#define CAN_CHANNEL IO_CAN_CHANNEL_0

#define DEBUG_CAN_CHANNEL IO_CAN_CHANNEL_1

#define BAUD_RATE 500

/* The system sends CAN messages from a FIFO buffer, this constant defines
 * its size. If a message is added to the full buffer, it is ignored.
 */
#define FIFO_BUFFER_SIZE 20

#define VCU_CONTROLS_CAN_ID 0xC0
#define VCU_INVERTER_SETTINGS_CAN_ID 0xC1
#define VCU_DEBUG_CAN_ID 0xDB

/**************************************************************************
 * Other
 ***************************************************************************/
 // 5 ms cycle time
 #define CYCLE_TIME MsToUs(5ul)

/* Application Database,
 * needed for TTC-Downloader
 */
APDB appl_db =
          { 0                      /* ubyte4 versionAPDB        */
          , {0}                    /* BL_T_DATE flashDate       */
                                   /* BL_T_DATE buildDate                   */
          , { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
          , 0                      /* ubyte4 nodeType           */
          , 0                      /* ubyte4 startAddress       */
          , 0                      /* ubyte4 codeSize           */
          , 0                      /* ubyte4 legacyAppCRC       */
          , 0                      /* ubyte4 appCRC             */
          , 1                      /* ubyte1 nodeNr             */
          , 0                      /* ubyte4 CRCInit            */
          , 0                      /* ubyte4 flags              */
          , 0                      /* ubyte4 hook1              */
          , 0                      /* ubyte4 hook2              */
          , 0                      /* ubyte4 hook3              */
          , APPL_START             /* ubyte4 mainAddress        */
          , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
          , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
          , 0                      /* ubyte4 legacyHeaderCRC    */
          , 0                      /* ubyte4 version            */
          , 500                    /* ubyte2 canBaudrate        */
          , 0                      /* ubyte1 canChannel         */
          , {0}                    /* ubyte1 reserved[8*4]      */
          , 0                      /* ubyte4 headerCRC          */
          };

void get_rtd(bool *rtd_val) {
    IO_DI_Get(IO_PIN_RTD, rtd_val);
}

void get_sdc(bool *sdc_val) {
    IO_DI_Get(IO_PIN_SDC, sdc_val);
}


// gets apps values, puts average pct travel into apps_pct_result
// if an error is detected, turns error to TRUE
void get_apps(ubyte2 *apps_pct_result, bool *error) {
    ubyte2 apps_1_val;
    bool apps_1_fresh;
    ubyte2 apps_2_val;
    bool apps_2_fresh;

    // get voltage values
    IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);
    IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

    bool apps_1_within_threshhold = (apps_1_val >= APPS_1_MIN_VOLTAGE) && (apps_1_val <= APPS_1_MAX_VOLTAGE);
    bool apps_2_within_threshhold = (apps_2_val >= APPS_2_MIN_VOLTAGE) && (apps_2_val <= APPS_2_MAX_VOLTAGE);

    ubyte4 implausibility_timestamp;
    ubyte2 apps_1_pct;
    ubyte2 apps_2_pct;
    sbyte2 difference;

    // check that both apps are within the treshhold
    if (apps_1_fresh && apps_1_within_threshhold &&
        apps_2_fresh && apps_2_within_threshhold) {

        apps_1_pct = VoltageToPctTravelApps1(apps_1_val);
        apps_2_pct = VoltageToPctTravelApps2(apps_2_val);

        difference = apps_1_pct - apps_2_pct;


        // check if there's an implausible deviation between the two
        if ((Abs(difference)) >= APPS_MIN_IMPLAUSIBLE_DEVIATION) {
            IO_RTC_StartTime(&implausibility_timestamp);

            // continuously check APPS for 100 ms to check if it stays implausible
            while (IO_RTC_GetTimeUS(implausibility_timestamp) <= IMPLAUSIBILITY_PERSISTENCE_PERIOD_US) {
                IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);
                IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

                apps_1_within_threshhold = (apps_1_val >= APPS_1_MIN_VOLTAGE) && (apps_1_val <= APPS_1_MAX_VOLTAGE);
                apps_2_within_threshhold = (apps_2_val >= APPS_2_MIN_VOLTAGE) && (apps_2_val <= APPS_2_MAX_VOLTAGE);

                // if the values are plausible
                if (apps_1_within_threshhold && apps_2_within_threshhold) {
                    apps_1_pct = VoltageToPctTravelApps1(apps_1_val);
                    apps_2_pct = VoltageToPctTravelApps2(apps_2_val);

                    difference = apps_1_pct - apps_2_pct;

                    // then return
                    if ((Abs(difference)) < APPS_MIN_IMPLAUSIBLE_DEVIATION) {
                        *apps_pct_result = ((apps_1_pct) + (apps_2_pct)) / 2;
                        *error = FALSE;
                        return;
                    }
                }
            }

            // if the values never became plausible, then indicate an error
            *apps_pct_result = 0;
            *error = TRUE;
            return;
        }


        // set the result to the average of the two
        *apps_pct_result = ((apps_1_pct) + (apps_2_pct)) / 2;
        *error = FALSE;
    } else {
        *error = TRUE;
        *apps_pct_result = 0;
    }
}

void get_bse(ubyte2 *bse_result, bool *error) {
    ubyte2 bse_val;
    bool bse_fresh;

    // get voltage from pin
    IO_ADC_Get(IO_PIN_BSE, &bse_val, &bse_fresh);

    // check if its in the treshhold
    bool bse_within_threshhold = (bse_val >= BSE_MIN_VOLTAGE) && (bse_val <= BSE_MAX_VOLTAGE);

    // error out if it isn't
    if (bse_fresh && bse_within_threshhold) {
        *bse_result = bse_val;
        *error = FALSE;
    } else {
        *bse_result = 0;
        *error = TRUE;
    }
}

void main (void)
{
    ubyte4 timestamp;

    enum VCU_State { NOT_READY, PLAYING_RTD_SOUND, DRIVING, ERRORED, APPS_5PCT_WAIT };

    /*******************************************/
    /*             INITIALIZATION              */
    /*******************************************/

    /* Initialize the IO driver (without safety functions) */
    IO_Driver_Init(NULL);

    /* Add your initialization code here            */
    /*  - Use the IO driver functions to initialize */
    /*    all required IOs and interfaces           */

    /* NOTE: turns 5v sensor supply 1 on */
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    /* CAN variables and initialization */
    ubyte1 handle_fifo_w;
    ubyte1 handle_tms_r;
    ubyte1 handle_inverter_voltage_r;
    ubyte1 handle_fifo_w_debug;

    /* can frame used to send torque requests to the inverter */
    IO_CAN_DATA_FRAME controls_can_frame;
    controls_can_frame.id = VCU_CONTROLS_CAN_ID;
    controls_can_frame.id_format = IO_CAN_STD_FRAME;
    controls_can_frame.length = 8;

    IO_CAN_DATA_FRAME inverter_settings_can_frame;
    inverter_settings_can_frame.id = VCU_INVERTER_SETTINGS_CAN_ID;
    inverter_settings_can_frame.id_format = IO_CAN_STD_FRAME;
    inverter_settings_can_frame.length = 8;
    for (int i = 0; i < 8; i++) {
        inverter_settings_can_frame.data[i] = 0;
    }

    // CAN frames for reading from tms and inverter voltage respectively
    IO_CAN_DATA_FRAME tms_read_can_frame;
    IO_CAN_DATA_FRAME inverter_voltage_read_can_frame;

    /* can frame used for debugging */
    IO_CAN_DATA_FRAME debug_can_frame;
    debug_can_frame.id = VCU_DEBUG_CAN_ID;
    debug_can_frame.id_format = IO_CAN_STD_FRAME;
    debug_can_frame.length = 8;
    for (int i = 0; i < 8; i++) {
        debug_can_frame.data[i] = 0;
    }

    /* initialize can channel and fifo buffer */
    IO_CAN_Init( CAN_CHANNEL
               , BAUD_RATE
               , 0
               , 0
               , 0);

    IO_CAN_Init( DEBUG_CAN_CHANNEL
               , BAUD_RATE
               , 0
               , 0
               , 0);

    IO_CAN_ConfigFIFO( &handle_fifo_w
    				 , CAN_CHANNEL
    				 , FIFO_BUFFER_SIZE
    				 , IO_CAN_MSG_WRITE
    				 , IO_CAN_STD_FRAME
    				 , VCU_CONTROLS_CAN_ID
    				 , 0);

    IO_CAN_ConfigMsg( &handle_tms_r
                     , CAN_CHANNEL
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , 1
                     , 0x1FFFFFFF);

    IO_CAN_ConfigMsg( &handle_inverter_voltage_r
                     , CAN_CHANNEL
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , 0xA7
                     , 0x1FFFFFFF);

    IO_CAN_ConfigFIFO( &handle_fifo_w_debug
    				 , DEBUG_CAN_CHANNEL
    				 , FIFO_BUFFER_SIZE
    				 , IO_CAN_MSG_WRITE
    				 , IO_CAN_STD_FRAME
    				 , VCU_CONTROLS_CAN_ID
    				 , 0);

    /* rtd with 10k pull-up resistor*/
    bool rtd_val;
    IO_DI_Init( IO_PIN_RTD,
                IO_DI_PU_10K );


    /* rtd variables */
    ubyte4 rtd_timestamp;
    bool just_entered_sound_state = TRUE;


    /* APPS 1 */
    IO_ADC_ChannelInit( IO_PIN_APPS_1,
                        IO_ADC_RATIOMETRIC,
                        0,
                        0,
                        IO_APPS_1_SUPPLY,
                        NULL );

    /* APPS 2 */
    IO_ADC_ChannelInit( IO_PIN_APPS_2,
                        IO_ADC_RATIOMETRIC,
                        0,
                        0,
                        IO_APPS_2_SUPPLY,
                        NULL );

    /* APPS in general */
    ubyte2 apps_pct_result;
    bool apps_error;

    /* BSE */
    IO_ADC_ChannelInit( IO_PIN_BSE,
                        IO_ADC_RATIOMETRIC,
                        0,
                        0,
                        IO_BSE_SUPPLY,
                        NULL );
    ubyte2 bse_result;
    bool bse_error;

    /* sdc with 10k pull-down resistor*/
    bool sdc_val;
    IO_DI_Init( IO_PIN_SDC,
                IO_DI_PD_10K );

    /* buzzer */
    IO_DO_Init( IO_PIN_BUZZER );

    /* VCU State */
    enum VCU_State current_state = NOT_READY;

    // sensor input is invalid during the first cycle
    bool first_cycle = TRUE;


    /* Voltage checking functionality */
    bool tms_voltage_read = FALSE;
    bool inverter_voltage_read = FALSE;
    bool voltage_match = FALSE;
    ubyte4 inverter_voltage = 0;
    ubyte4 tms_voltage = 0;
    IO_ErrorType x;
    IO_ErrorType y;

    ubyte4 torque;
    ubyte4 d0;
    ubyte4 d1;



    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/

    /* main loop, executed periodically with a
     * defined cycle time (here: CYCLE_TIME_MS ms)
     */
    while (1)
    {
        /* get a timestamp to implement the cycle time */
        IO_RTC_StartTime(&timestamp);
        /* Task begin function for IO Driver
         * This function needs to be called at
         * the beginning of every SW cycle
         */
        IO_Driver_TaskBegin();


        // during the first cycle, every sensor input is invalid, so skip it
        if (first_cycle) {
            first_cycle = FALSE;
        } else {

            if (current_state == NOT_READY) {
                get_rtd(&rtd_val);
                get_bse(&bse_result, &bse_error);
                // transitions
                // rtd on and brakes engaged -> play rtd sound
                if (rtd_val == RTD_ON && voltage_match /*&& (!bse_error && bse_result > BRAKES_ENGAGED_BSE_THRESHOLD)*/) {
                    // rtd on -> switch to playing rtd sound
                    current_state = PLAYING_RTD_SOUND;
                } else {
                    // if you don't transition away from this state, send a 0
                    // torque message to the motor with 0 torque
                    for (int i = 0; i < 8; i++) {
                        controls_can_frame.data[i] = 0;
                    }
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                    IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);
                }

            } else if (current_state == PLAYING_RTD_SOUND) {

                if (just_entered_sound_state) {
                    // when this state has just been entered, turn on the buzzer and a timer
                    IO_RTC_StartTime(&rtd_timestamp);
                    just_entered_sound_state = FALSE;

                    IO_DO_Set(IO_PIN_BUZZER, TRUE);

                    inverter_settings_can_frame.data[0] = 20;
                    inverter_settings_can_frame.data[1] = 0;
                    inverter_settings_can_frame.data[2] = 1;
                    inverter_settings_can_frame.data[3] = 0;
                    inverter_settings_can_frame.data[4] = 0;
                    inverter_settings_can_frame.data[5] = 0;
                    inverter_settings_can_frame.data[6] = 0;
                    inverter_settings_can_frame.data[7] = 0;

                    IO_CAN_WriteFIFO(handle_fifo_w, &inverter_settings_can_frame, 1);
                    IO_CAN_WriteFIFO(handle_fifo_w_debug, &inverter_settings_can_frame, 1);

                } else if (!just_entered_sound_state && IO_RTC_GetTimeUS(rtd_timestamp) > RTD_SOUND_DURATION) {
                    // once the timer runs out, enter the new state (driving) and turn off the buzzer
                    current_state = DRIVING;
                    just_entered_sound_state = TRUE;
                    IO_DO_Set(IO_PIN_BUZZER, TRUE);
                }

                // keep sending 0 torque messages to the inverter in this state
                for (int i = 0; i < 8; i++) {
                    controls_can_frame.data[i] = 0;
                }
                IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);

            } else if (current_state == DRIVING) {
                // get the rtd, apps, and bse
                get_rtd(&rtd_val);
                get_sdc(&sdc_val);
                get_apps(&apps_pct_result, &apps_error);
                get_bse(&bse_result, &bse_error);

                //*************************************
                bse_error = 0;
                //*************************************


                // transitions
                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;
                } else if (apps_error /*|| bse_error || (sdc_val == SDC_OFF)*/) {
                    current_state = ERRORED;
                } else if (0 /*bse_result > BRAKES_ENGAGED_BSE_THRESHOLD && apps_pct_result >= APPS_THRESHHOLD_BRAKE_PLAUSIBILITY*/) {
                    current_state = APPS_5PCT_WAIT;
                } else {
                    // no transition into another state -> send controls message
                    torque = PCT_TRAVEL_TO_TORQUE(apps_pct_result);
                    d0 = (torque * 10) % 256;
                    d1 = (torque * 10) / 256;
                    controls_can_frame.data[0] = d0;
                    controls_can_frame.data[1] = d1;
                    controls_can_frame.data[2] = 0;
                    controls_can_frame.data[3] = 0;
                    // forward direction
                    controls_can_frame.data[4] = 1;
                    // enable inverter
                    controls_can_frame.data[5] = 1;
                    controls_can_frame.data[6] = 0;
                    controls_can_frame.data[7] = 0;
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                    IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);
                }
            } else if (current_state == ERRORED) {
                get_rtd(&rtd_val);

                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;
                }

                // continue sending 0 torque messages to the inverter in this state
                for (int i = 0; i < 8; i++) {
                    controls_can_frame.data[i] = 0;
                }
                IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);
            } else if (current_state == APPS_5PCT_WAIT) {
                // when brakes are engaged and apps > 25%, the car goes into this state
                get_rtd(&rtd_val);
                get_sdc(&sdc_val);
                get_apps(&apps_pct_result, &apps_error);
                get_bse(&bse_result, &bse_error);

                //*************************************
                bse_error = 0;
                //*************************************

                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;

                } else if (apps_error || bse_error /*|| (sdc_val == SDC_OFF)*/) {
                    current_state = ERRORED;
                } else if (apps_pct_result <= APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY) {
                    // go back to driving when apps <= 5%
                    current_state = DRIVING;
                } else {
                    // no transition into another state -> send 0 torque message
                    for (int i = 0; i < 8; i++) {
                        controls_can_frame.data[i] = 0;
                    }
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                    IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);
                }

            }

            // send out a debug frame with the current state
            debug_can_frame.data[0] = current_state;
            // get_apps(&apps_pct_result, &apps_error);
            // get_bse(&bse_result, &bse_error);

            ubyte2 apps_1_val;
            bool apps_1_fresh;
            ubyte2 apps_2_val;
            bool apps_2_fresh;

            ubyte2 bse_val;
            bool bse_fresh;

            /*
            if (IO_CAN_MsgStatus(handle_tms_r) == IO_E_OK) {
                IO_CAN_ReadMsg(handle_tms_r, &tms_read_can_frame);
                debug_can_frame.data[5] = tms_read_can_frame.data[0];
            }

            if (IO_CAN_MsgStatus(handle_inverter_voltage_r) == IO_E_OK) {
                IO_CAN_ReadMsg(handle_inverter_voltage_r, &inverter_voltage_read_can_frame);
                debug_can_frame.data[6] = inverter_voltage_read_can_frame.data[0];
            }
            */


            IO_ADC_Get(IO_PIN_APPS_1, &apps_1_val, &apps_1_fresh);
            IO_ADC_Get(IO_PIN_APPS_2, &apps_2_val, &apps_2_fresh);

            IO_ADC_Get(IO_PIN_BSE, &bse_val, &bse_fresh);

            // debug_can_frame.data[1] = apps_1_val >> 8;
            // debug_can_frame.data[2] = apps_1_val & 0xFF;

            // debug_can_frame.data[3] = apps_2_val >> 8;
            // debug_can_frame.data[4] = apps_2_val & 0xFF;

            // debug_can_frame.data[5] = voltage_match;
            // debug_can_frame.data[6] = tms_voltage_read;
            // debug_can_frame.data[7] = inverter_voltage_read;


            x = IO_CAN_MsgStatus(handle_tms_r);
            if (x == IO_E_OK || x == IO_E_CAN_OLD_DATA) {
                IO_CAN_ReadMsg(handle_tms_r, &tms_read_can_frame);
                debug_can_frame.data[1] = tms_read_can_frame.data[0];
                tms_voltage = (tms_read_can_frame.data[5] << 8) | (tms_read_can_frame.data[4]);
                tms_voltage = tms_voltage / 10;
            }

            y = IO_CAN_MsgStatus(handle_inverter_voltage_r);
            if (y == IO_E_OK || y == IO_E_CAN_OLD_DATA) {
                IO_CAN_ReadMsg(handle_inverter_voltage_r, &inverter_voltage_read_can_frame);
                debug_can_frame.data[2] = inverter_voltage_read_can_frame.data[0];
                inverter_voltage = (inverter_voltage_read_can_frame.data[1] << 8) | (inverter_voltage_read_can_frame.data[0]);
                inverter_voltage = inverter_voltage / 10;
            }

            if ((tms_voltage < inverter_voltage + 10) &&
                (tms_voltage > inverter_voltage - 10) && inverter_voltage >= 290) {
                    voltage_match = TRUE;
            } else {
                    voltage_match = FALSE;
            }



            IO_CAN_WriteFIFO(handle_fifo_w_debug, &debug_can_frame, 1);
            IO_CAN_WriteFIFO(handle_fifo_w, &debug_can_frame, 1);
            IO_CAN_WriteFIFO(handle_fifo_w_debug, &controls_can_frame, 1);
        }

        /* Task end function for IO Driver
         * This function needs to be called at
         * the end of every SW cycle
         */
        IO_Driver_TaskEnd();
        /* wait until the cycle time is over */

        /* NOTE:
         * If the code between the TaskBegin and TaskEnd doesn't take
         * 10 milliseconds, this code delays the end of the cycle so it lasts
         * 10 milliseconds
         */
        while (IO_RTC_GetTimeUS(timestamp) < CYCLE_TIME);
    }
}
