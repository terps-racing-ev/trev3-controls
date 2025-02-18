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

#include "apps.h"
#include "bse.h"
#include "tsil.h"
#include "utilities.h"
#include "debug_defines.h"


/**************************************************************************
 * Sensor Pins
 ***************************************************************************/
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
/* percent to torque algorithm */
// #define PCT_TRAVEL_TO_TORQUE(val) ((int)(((float)(val) / 100.0) * 112.0))
#define PCT_TRAVEL_FOR_MAX_TORQUE 80

#define CONTINUOUS_TORQUE_MAX 200
/**************************************************************************
 * SDC Settings
 ***************************************************************************/
#define SDC_ON 1

/**********************************************************/
#define SDC_OFF 0 //0
/**********************************************************/

/**************************************************************************
 * Brake Light
 ***************************************************************************/
 #define BRAKE_LIGHT_PIN IO_DO_02


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

#define TMS_SUMMARY_1_CAN_ID 0x1
#define TMS_SUMMARY_2_CAN_ID 0x2


#define MOTOR_INFO_CAN_ID 0xA5
#define VOLTAGE_INFO_CAN_ID 0xA7
#define CURRENT_INFO_CAN_ID 0xA6

#define INVERTER_PACK_VOLT_LO 0
#define INVERTER_PACK_VOLT_HI 1

#define INVERTER_MOTOR_SPEED_LO 2
#define INVERTER_MOTOR_SPEED_HI 3

#define ORION_CAN_ID 0x6B2
#define ORION_IMD_STATUS_INDEX 0
#define ORION_BMS_STATUS_INDEX 1

// 10 second CAN timeout on Orion
#define ORION_CAN_TIMEOUT_S 10
#define ORION_CAN_TIMEOUT_US MsToUs(ORION_CAN_TIMEOUT_S * 1000)

// 3 second period for ignoring Orion
#define ORION_IGNORE_PERIOD_S 3
#define ORION_IGNORE_PERIOD_US MsToUs(ORION_IGNORE_PERIOD_S * 1000)

/**************************************************************************
 * Other
 ***************************************************************************/
 // 5 ms cycle time
 #define CYCLE_TIME MsToUs(5ul)

 #define PRECHARGE_VOLTAGE_THRESHHOLD 268
 #define TORQUE_LIMITING_RPM_THRESHHOLD 80
 #define RPM_BASED_TORQUE_LIMIT 15

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
    if (IGNORE_RTD_SWITCH) {
        *rtd_val = RTD_ON;
    } else {
        IO_DI_Get(IO_PIN_RTD, rtd_val);
    }    
}

void get_sdc(bool *sdc_val) {
    if (IGNORE_SDC_OFF) {
        *sdc_val = SDC_OFF;
    } else {
        IO_DI_Get(IO_PIN_SDC, sdc_val);
    }
}

void read_can_msg(ubyte1* handle_r, IO_CAN_DATA_FRAME* dst_data_frame, bool *msg_received, ubyte4 id, ubyte1 channel) {
    IO_ErrorType can_read_error;

    *msg_received = FALSE;

    can_read_error = IO_CAN_MsgStatus(*handle_r);

    if (can_read_error == IO_E_OK) {
        IO_CAN_ReadMsg(*handle_r, dst_data_frame);
        *msg_received = TRUE;
    } else if (can_read_error == IO_E_CAN_OVERFLOW) {
        IO_CAN_DeInitHandle(*handle_r);

        IO_CAN_ConfigMsg( handle_r
                , channel
                , IO_CAN_MSG_READ
                , IO_CAN_STD_FRAME
                , id
                , 0x1FFFFFFF);
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

    return ((ubyte4)(((float)(pct_travel) / PCT_TRAVEL_FOR_MAX_TORQUE) * 200.0));
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
    ubyte1 handle_tms_summary_1_r;
    ubyte1 handle_tms_summary_2_r;
    ubyte1 handle_inverter_motor_info_r;
    ubyte1 handle_inverter_voltage_info_r;
    ubyte1 handle_inverter_current_info_r;
    ubyte1 handle_orion_r;

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
    IO_CAN_DATA_FRAME tms_summary_1_can_frame;
    IO_CAN_DATA_FRAME tms_summary_2_can_frame;


    // CAN frame for reading from inverter
    IO_CAN_DATA_FRAME inverter_motor_info_can_frame;
    IO_CAN_DATA_FRAME inverter_voltage_info_can_frame;
    IO_CAN_DATA_FRAME inverter_current_info_can_frame;


    // CAN frame for orion
    IO_CAN_DATA_FRAME orion_can_frame;

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

    IO_CAN_ConfigMsg( &handle_tms_summary_1_r
                     , CAN_CHANNEL
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , TMS_SUMMARY_1_CAN_ID
                     , 0x1FFFFFFF);

    IO_CAN_ConfigMsg( &handle_tms_summary_2_r
                     , CAN_CHANNEL
                     , IO_CAN_MSG_READ
                     , IO_CAN_STD_FRAME
                     , TMS_SUMMARY_2_CAN_ID
                     , 0x1FFFFFFF);

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

    IO_CAN_ConfigMsg( &handle_inverter_current_info_r
                 , CAN_CHANNEL
                 , IO_CAN_MSG_READ
                 , IO_CAN_STD_FRAME
                 , CURRENT_INFO_CAN_ID
                 , 0x1FFFFFFF);

    IO_CAN_ConfigMsg( &handle_orion_r
                 , CAN_CHANNEL
                 , IO_CAN_MSG_READ
                 , IO_CAN_STD_FRAME
                 , ORION_CAN_ID
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
    ubyte2 apps_pct_result = 0;
    bool apps_error = FALSE;
    ubyte1 num_errors = 0;

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
    bool tms_summary_1_message_received = FALSE;
    bool tms_summary_2_message_received = FALSE;

    // variables to hold torque
    ubyte4 torque = 0;
    ubyte4 limited_torque = 0;
    ubyte4 d0 = 0;
    ubyte4 d1 = 0;
    ubyte4 motor_rpm_torque_limit = CONTINUOUS_TORQUE_MAX;

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

    //inverter current info received
    bool current_info_message_received = FALSE;

    // orion message received
    bool orion_message_received = FALSE;
    bool orion_message_received_once = FALSE;
    bool orion_good = TRUE;

    // lights
    IO_DO_Init( TSIL_GREEN_PIN );
    IO_DO_Init( TSIL_RED_PIN );

    // make sure light is green
    set_light_to(SOLID_GREEN);

    bool been_ignore_period_since_start = FALSE;
    ubyte4 time_since_start;
    ubyte4 orion_can_timeout;

    IO_RTC_StartTime(&time_since_start);
    IO_RTC_StartTime(&orion_can_timeout);

    // brake lights
    IO_DO_Init( BRAKE_LIGHT_PIN );




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

        // read tms messages
        read_can_msg(&handle_tms_summary_1_r, &tms_summary_1_can_frame, &tms_summary_1_message_received, TMS_SUMMARY_1_CAN_ID, CAN_CHANNEL);
        read_can_msg(&handle_tms_summary_2_r, &tms_summary_2_can_frame, &tms_summary_2_message_received, TMS_SUMMARY_2_CAN_ID, CAN_CHANNEL);

        // read current info from inverter
        read_can_msg(&handle_inverter_current_info_r, &inverter_current_info_can_frame, &current_info_message_received, CURRENT_INFO_CAN_ID, CAN_CHANNEL);

        // read message from inverter with motor speed and update motor speed accordingly
        read_can_msg(&handle_inverter_motor_info_r, &inverter_motor_info_can_frame, &motor_info_message_received, MOTOR_INFO_CAN_ID, CAN_CHANNEL);
        if (motor_info_message_received) {
            // d0 and d1 are used for the outgoing CAN message
            last_speed_d0 = inverter_motor_info_can_frame.data[INVERTER_MOTOR_SPEED_LO];
            last_speed_d1 = inverter_motor_info_can_frame.data[INVERTER_MOTOR_SPEED_HI];
            last_speed = (last_speed_d1 << 8) | last_speed_d0;
            motor_speed_updated_once = TRUE;
        }


        // read message from inverter with voltage and update pack voltage accordingly
        read_can_msg(&handle_inverter_voltage_info_r, &inverter_voltage_info_can_frame, &voltage_info_message_received, VOLTAGE_INFO_CAN_ID, CAN_CHANNEL);

        // read message from orion
        read_can_msg(&handle_orion_r, &orion_can_frame, &orion_message_received, ORION_CAN_ID, CAN_CHANNEL);

        if (orion_message_received) {
            // reset the timeout if a message has been received
            IO_RTC_StartTime(&orion_can_timeout);
            orion_message_received_once = TRUE;
        }

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
                get_apps(&apps_pct_result, &apps_error, &num_errors);

                // transitions
                // rtd on and brakes engaged -> play rtd sound
                if (rtd_val == RTD_ON && (IGNORE_RTD_BRAKES || (!bse_error && bse_result > BRAKES_ENGAGED_BSE_THRESHOLD)) && !apps_error && !(sdc_val == SDC_OFF)) {
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


                    // clear Inverter faults
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
                get_apps(&apps_pct_result, &apps_error, &num_errors);
                get_bse(&bse_result, &bse_error);

                // transitions
                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;
                } else if (apps_error || bse_error || (sdc_val == SDC_OFF)) {
                    current_state = ERRORED;
                } else if (!(IGNORE_BRAKE_PLAUSIBILITY) && bse_result > BRAKE_PLAUSIBILITY_BRAKES_ENGAGED_BSE_THRESHOLD && apps_pct_result >= APPS_THRESHHOLD_BRAKE_PLAUSIBILITY) {
                    current_state = APPS_5PCT_WAIT;
                } else {
                    // no transition into another state -> send controls message
                    torque = pct_travel_to_torque(apps_pct_result);

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
                get_apps(&apps_pct_result, &apps_error, &num_errors);
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

            // echo tms messages
            if (tms_summary_1_message_received) {
            	IO_CAN_WriteFIFO(handle_fifo_w_debug, &tms_summary_1_can_frame, 1);
            }
            if (tms_summary_2_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &tms_summary_2_can_frame, 1);
            }

            // echo motor info message
            if (motor_info_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &inverter_motor_info_can_frame, 1);
            }

            if (voltage_info_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &inverter_voltage_info_can_frame, 1);
            }


            if (current_info_message_received) {
                IO_CAN_WriteFIFO(handle_fifo_w_debug, &inverter_current_info_can_frame, 1);
            }


            // code to control tsil lights 

            // check if it's been "ignore period" seconds since start
            if (been_ignore_period_since_start == FALSE) {
                if (IO_RTC_GetTimeUS(time_since_start) > ORION_IGNORE_PERIOD_US) {
                    been_ignore_period_since_start = TRUE;
                }
            }

            // only check CAN message if it's been "ignore period" seconds since start
            if (been_ignore_period_since_start == TRUE) {
                // if CAN timeout, set light to blinking red
                if (IO_RTC_GetTimeUS(orion_can_timeout) > ORION_CAN_TIMEOUT_US) {
                    set_light_to(BLINKING_RED);
                } else {
                    if (orion_message_received_once == TRUE) {
                        get_sdc(&sdc_val);

                        // if SDC is good and BMS is good and IMD is good
                        // set lights to green
                        if (orion_can_frame.data[ORION_BMS_STATUS_INDEX] &&
                            orion_can_frame.data[ORION_IMD_STATUS_INDEX] && 
                            (sdc_val != SDC_OFF)) {
                                set_light_to(SOLID_GREEN);
                        
                        // if either IMD or BMS is bad set lights to red
                        // (we don't check SDC)
                        } else if (!orion_can_frame.data[ORION_BMS_STATUS_INDEX] || 
                                   !orion_can_frame.data[ORION_IMD_STATUS_INDEX]) {
                                set_light_to(BLINKING_RED);
                        }
                    }
                }
            }

            // run the lights (function call needed for lights to blink)
            do_lights_action();


            // code for brake light
            get_bse(&bse_result, &bse_error);

            if (!bse_error && (bse_result > BRAKES_ENGAGED_BSE_THRESHOLD)) {
                IO_DO_Set(BRAKE_LIGHT_PIN, TRUE);
            } else {
                IO_DO_Set(BRAKE_LIGHT_PIN, FALSE);
            }


            // send debug message
            debug_can_frame.id = 0xDB;
            debug_can_frame.data[0] = apps_pct_result;

            debug_can_frame.data[1] = d0;
            debug_can_frame.data[2] = d1;

            // if the moving average filter is being used then this is filtered
            debug_can_frame.data[3] = bse_result & 0xFF;
            debug_can_frame.data[4] = bse_result >> 8;

            debug_can_frame.data[5] = current_state;

            debug_can_frame.data[6] = last_speed_d0;
            debug_can_frame.data[7] = last_speed_d1;

            IO_CAN_WriteFIFO(handle_fifo_w, &debug_can_frame, 1);
            IO_CAN_WriteFIFO(handle_fifo_w_debug, &debug_can_frame, 1);

            ubyte2 apps_1_val;
            bool apps_1_fresh;
            ubyte2 apps_2_val;
            bool apps_2_fresh;

            ubyte2 bse_val;
            bool bse_fresh;

            // send another debug message
            // get voltage values
            IO_ADC_Get(IO_PIN_BSE, &bse_val, &bse_fresh);

            debug_can_frame.id = 0xDC;

            if (apps_error) {
            debug_can_frame.data[0] = apps_error;
            } else {
            debug_can_frame.data[0] = 0;
            }

            if (bse_error) {
            debug_can_frame.data[1] = 1;
            } else {
            debug_can_frame.data[1] = 0;
            }

            apps_1_val = get_filtered_apps1_voltage();
            apps_2_val = get_filtered_apps2_voltage();
            debug_can_frame.data[2] = apps_1_val & 0xFF;
            debug_can_frame.data[3] = apps_1_val >> 8;

            debug_can_frame.data[4] = apps_2_val & 0xFF;
            debug_can_frame.data[5] = apps_2_val >> 8;

            // unfiltered bse
            debug_can_frame.data[6] = bse_val & 0xFF;
            debug_can_frame.data[7] = bse_val >> 8;
            

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
