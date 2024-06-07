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

/* BSE -> pin 147 (aka adc 5V 7)*/
#define IO_PIN_BSE IO_ADC_5V_07
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

//***************************************** used to be 25
#define APPS_THRESHHOLD_BRAKE_PLAUSIBILITY 25
//*******************************************************

#define APPS_THRESHHOLD_REESTABLISH_PLAUSIBILITY 5

/* apps 1 */

#define APPS_1_MAX_VOLTAGE 3550

#define APPS_1_MIN_VOLTAGE 850

#define APPS_1_VOLTAGE_RANGE (APPS_1_MAX_VOLTAGE - APPS_1_MIN_VOLTAGE)

/* apps 2 */

#define APPS_2_MAX_VOLTAGE 4750

#define APPS_2_MIN_VOLTAGE 930

#define APPS_2_VOLTAGE_RANGE (APPS_2_MAX_VOLTAGE - APPS_2_MIN_VOLTAGE)


/* bse */
#define BSE_MAX_VOLTAGE 4510

#define BSE_MIN_VOLTAGE 490

#define BRAKES_ENGAGED_BSE_THRESHOLD 550

#define BRAKE_PLAUSIBILITY_BRAKES_ENGAGED_BSE_THRESHOLD 550

/* percent to torque algorithm */
// #define PCT_TRAVEL_TO_TORQUE(val) ((int)(((float)(val) / 100.0) * 112.0))
#define PCT_TRAVEL_FOR_MAX_TORQUE 80

#define CONTINUOUS_TORQUE_MAX 112
/**************************************************************************
 * SDC Settings
 ***************************************************************************/
#define SDC_ON 1

/**********************************************************/
#define SDC_OFF 0
/**********************************************************/

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

#define TMS_SUMMARY_CAN_ID 0x13
#define TMS_PACK_VOLT_LO 2
#define TMS_PACK_VOLT_HI 3

#define MOTOR_INFO_CAN_ID 0xA5
#define VOLTAGE_INFO_CAN_ID 0xA7

#define INVERTER_PACK_VOLT_LO 0
#define INVERTER_PACK_VOLT_HI 1

#define INVERTER_MOTOR_SPEED_LO 2
#define INVERTER_MOTOR_SPEED_HI 3

#define NUM_MODULES 6
#define NUM_MODULE_FRAMES (NUM_MODULES * 3)

/**************************************************************************
 * Other
 ***************************************************************************/
 // 5 ms cycle time
 #define CYCLE_TIME MsToUs(5ul)

 #define PRECHARGE_VOLTAGE_THRESHHOLD 268
 #define TORQUE_LIMITING_RPM_THRESHHOLD 25
 #define RPM_BASED_TORQUE_LIMIT 30

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


void read_can_msg(ubyte1 handle_r, IO_CAN_DATA_FRAME* dst_data_frame, bool *msg_received) {
    IO_ErrorType can_read_error;

    *msg_received = FALSE;

    can_read_error = IO_CAN_MsgStatus(handle_r);

    if (can_read_error == IO_E_OK) {
        IO_CAN_ReadMsg(handle_r, dst_data_frame);
        *msg_received = TRUE;
    }

}

void read_tms_messages(ubyte1 summary_handle_r, IO_CAN_DATA_FRAME* summary_dst_data_frame, bool* summary_msg_received,
                       ubyte1* module_handles, IO_CAN_DATA_FRAME* module_data_frames, bool* module_msg_received) {
    int i = 0;
    read_can_msg(summary_handle_r, summary_dst_data_frame, summary_msg_received);

    for (i = 0; i < NUM_MODULE_FRAMES; i++) {
        read_can_msg(module_handles[i], &(module_data_frames[i]), &(module_msg_received[i]));
    }
}

double voltage_to_torque_limit (ubyte2 pack_voltage) {
    if (pack_voltage < PRECHARGE_VOLTAGE_THRESHHOLD) {
        return 0;
    } else {
        return ((double) (pack_voltage)) * -0.3255 + 231.25;
    }

}

ubyte4 pct_travel_to_torque (ubyte4 pct_travel) {
    if (pct_travel >= PCT_TRAVEL_FOR_MAX_TORQUE) {
         return CONTINUOUS_TORQUE_MAX;
    }

    return ((ubyte4)(((float)(pct_travel) / PCT_TRAVEL_FOR_MAX_TORQUE) * 112.0));
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
    // transmitting
    ubyte1 handle_fifo_w;
    ubyte1 handle_fifo_w_debug;

    // receiving
    ubyte1 handle_tms_summary_r;
    ubyte1 handles_tms_module_info_r[NUM_MODULE_FRAMES];
    ubyte1 handle_inverter_motor_info_r;
    ubyte1 handle_inverter_voltage_info_r;

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

    // CAN frames for reading from tms
    IO_CAN_DATA_FRAME tms_summary_can_frame;
    IO_CAN_DATA_FRAME tms_module_can_frames[NUM_MODULE_FRAMES];

    // CAN frame for reading from inverter
    IO_CAN_DATA_FRAME inverter_motor_info_can_frame;
    IO_CAN_DATA_FRAME inverter_voltage_info_can_frame;

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

    IO_CAN_ConfigMsg( &handle_tms_summary_r
                     , CAN_CHANNEL
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , TMS_SUMMARY_CAN_ID
                     , 0x1FFFFFFF);

    // configure the 18 handles for the module info (3 for each module)
    for (int i = 0; i < NUM_MODULE_FRAMES; i++) {
        IO_CAN_ConfigMsg( &handles_tms_module_info_r[i]
                         , CAN_CHANNEL
                         , IO_CAN_MSG_READ
                         , IO_CAN_STD_FRAME
                         , (i + 1)
                         , 0x1FFFFFFF);
    }

    IO_CAN_ConfigMsg( &handle_inverter_motor_info_r
                 , CAN_CHANNEL
                 , IO_CAN_MSG_READ
                 , IO_CAN_STD_FRAME
                 , MOTOR_INFO_CAN_ID
                 , 0x1FFFFFFF);

    IO_CAN_ConfigMsg( &handle_inverter_voltage_info_r
                 , CAN_CHANNEL
                 , IO_CAN_MSG_READ
                 , IO_CAN_STD_FRAME
                 , VOLTAGE_INFO_CAN_ID
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
    bool tms_summary_message_received = FALSE;
    bool tms_module_message_received[NUM_MODULE_FRAMES] = {FALSE};
    int i;

    ubyte4 torque = 0;
    ubyte4 limited_torque = 0;
    ubyte4 d0 = 0;
    ubyte4 d1 = 0;

    // whether a new motor info message has been received since the last time
    bool motor_info_message_received = FALSE;
    // last received motor speed
    ubyte1 last_speed_d0 = 0;
    ubyte1 last_speed_d1 = 0;
    ubyte2 last_speed = 0;
    // whether any motor info message has been received
    bool motor_speed_updated_once = FALSE;

    // whether a new voltage info message has been received since the last time
    bool voltage_info_message_received = FALSE;
    // last received pack voltage
    ubyte2 pack_voltage = 0;
    // whether any voltage info has been received
    bool pack_voltage_updated_once = FALSE;


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

        // read messages from tms
        read_tms_messages(handle_tms_summary_r, &tms_summary_can_frame, &tms_summary_message_received,
                          handles_tms_module_info_r, tms_module_can_frames, tms_module_message_received);

        // read message from inverter with motor speed and update motor speed accordingly
        read_can_msg(handle_inverter_motor_info_r, &inverter_motor_info_can_frame, &motor_info_message_received);
        if (motor_info_message_received) {
            // d0 and d1 are used for the outgoing CAN message
            last_speed_d0 = inverter_motor_info_can_frame.data[INVERTER_MOTOR_SPEED_LO];
            last_speed_d1 = inverter_motor_info_can_frame.data[INVERTER_MOTOR_SPEED_HI];
            last_speed = (last_speed_d1 << 8) | last_speed_d0;
            motor_speed_updated_once = TRUE;
        }


        // read message from inverter with voltage and update pack voltage accordingly
        read_can_msg(handle_inverter_voltage_info_r, &inverter_voltage_info_can_frame, &voltage_info_message_received);
        if (voltage_info_message_received == TRUE) {
            pack_voltage = ((inverter_voltage_info_can_frame.data[INVERTER_PACK_VOLT_HI] << 8) | inverter_voltage_info_can_frame.data[INVERTER_PACK_VOLT_LO]) / 10;
            pack_voltage_updated_once = TRUE;
        }

        d0 = 0;
        d1 = 0;
        torque = 0;
        apps_pct_result = 0;
        bse_result = 0;

        // during the first cycle, every sensor input is invalid, so skip it
        if (first_cycle) {
            first_cycle = FALSE;
        } else {

            if (current_state == NOT_READY) {
                get_rtd(&rtd_val);
                get_bse(&bse_result, &bse_error);
                get_sdc(&sdc_val);
                get_apps(&apps_pct_result, &apps_error);

                // transitions
                // rtd on and brakes engaged -> play rtd sound
                if (rtd_val == RTD_ON && (!bse_error && bse_result > BRAKES_ENGAGED_BSE_THRESHOLD) && !apps_error && !(sdc_val == SDC_OFF)) {
                    // rtd on -> switch to playing rtd sound
                    current_state = PLAYING_RTD_SOUND;
                } else {
                    // if you don't transition away from this state, send a 0
                    // torque message to the motor with 0 torque
                    for (int i = 0; i < 8; i++) {
                        controls_can_frame.data[i] = 0;
                    }
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                    pack_voltage_updated_once = FALSE;
                }

            } else if (current_state == PLAYING_RTD_SOUND) {

                if (just_entered_sound_state) {
                    // when this state has just been entered, turn on the buzzer and a timer
                    IO_RTC_StartTime(&rtd_timestamp);
                    just_entered_sound_state = FALSE;

                    IO_DO_Set(IO_PIN_BUZZER, TRUE);


                    // clear Inverter faults (
                    inverter_settings_can_frame.data[0] = 20;
                    inverter_settings_can_frame.data[1] = 0;
                    inverter_settings_can_frame.data[2] = 1;
                    inverter_settings_can_frame.data[3] = 0;
                    inverter_settings_can_frame.data[4] = 0;
                    inverter_settings_can_frame.data[5] = 0;
                    inverter_settings_can_frame.data[6] = 0;
                    inverter_settings_can_frame.data[7] = 0;

                    IO_CAN_WriteFIFO(handle_fifo_w, &inverter_settings_can_frame, 1);

                } else if (!just_entered_sound_state && IO_RTC_GetTimeUS(rtd_timestamp) > RTD_SOUND_DURATION) {
                    // once the timer runs out, enter the new state (driving) and turn off the buzzer
                    current_state = DRIVING;
                    just_entered_sound_state = TRUE;
                    IO_DO_Set(IO_PIN_BUZZER, FALSE);
                }

                // keep sending 0 torque messages to the inverter in this state
                for (int i = 0; i < 8; i++) {
                    controls_can_frame.data[i] = 0;
                }
                IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);

            } else if (current_state == DRIVING) {
                // get the rtd, apps, and bse
                get_rtd(&rtd_val);
                get_sdc(&sdc_val);
                get_apps(&apps_pct_result, &apps_error);
                get_bse(&bse_result, &bse_error);

                // transitions
                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;
                } else if (apps_error || bse_error || (sdc_val == SDC_OFF)) {
                    current_state = ERRORED;
                } else if (bse_result > BRAKE_PLAUSIBILITY_BRAKES_ENGAGED_BSE_THRESHOLD && apps_pct_result >= APPS_THRESHHOLD_BRAKE_PLAUSIBILITY) {
                    current_state = APPS_5PCT_WAIT;
                } else {
                    // no transition into another state -> send controls message
                    torque = pct_travel_to_torque(apps_pct_result);

                    // limit torque based on pack voltage
                    if (pack_voltage_updated_once) {
                        limited_torque = (ubyte4) (voltage_to_torque_limit(pack_voltage));

                        if (torque > limited_torque) {
                            torque = limited_torque;
                        }
                    }

                    // limit torque based on RPM
                    if (motor_speed_updated_once) {
                        if (last_speed < TORQUE_LIMITING_RPM_THRESHHOLD &&
                            torque > RPM_BASED_TORQUE_LIMIT) {
                                torque = RPM_BASED_TORQUE_LIMIT;
                            }
                    }

                    // limit torque based on continuous torque max
                    if (torque > CONTINUOUS_TORQUE_MAX) {
                       torque = CONTINUOUS_TORQUE_MAX;
                    }

                    if (torque < 0) {
                        torque = 0;
                    }


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
            } else if (current_state == APPS_5PCT_WAIT) {
                // when brakes are engaged and apps > 25%, the car goes into this state
                get_rtd(&rtd_val);
                get_sdc(&sdc_val);
                get_apps(&apps_pct_result, &apps_error);
                get_bse(&bse_result, &bse_error);

                //*************************************
                // bse_error = 0;
                //*************************************

                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;

                } else if (apps_error || bse_error || (sdc_val == SDC_OFF)) {
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
                }

            }
            /*
            if (tms_summary_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &tms_summary_can_frame, 1);
            }
            */

            for (i = 0; i < NUM_MODULE_FRAMES; i++) {
                if (tms_module_message_received[i]) {
                    IO_CAN_WriteFIFO(handle_fifo_w_debug, &(tms_module_can_frames[i]), 1);
                }
            }

            if (voltage_info_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &inverter_voltage_info_can_frame, 1);
            }

            debug_can_frame.data[0] = apps_pct_result;

            debug_can_frame.data[1] = d0;
            debug_can_frame.data[2] = d1;

            debug_can_frame.data[3] = bse_result & 0xFF;
            debug_can_frame.data[4] = bse_result >> 8;

            debug_can_frame.data[5] = current_state;

            debug_can_frame.data[6] = last_speed_d0;
            debug_can_frame.data[7] = last_speed_d1;

            IO_CAN_WriteFIFO(handle_fifo_w, &debug_can_frame, 1);
            IO_CAN_WriteFIFO(handle_fifo_w_debug, &debug_can_frame, 1);

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
