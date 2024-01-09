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

/* APPS 2 -> pin 140 (aka adc 5V 1)*/
#define IO_PIN_APPS_2 IO_ADC_5V_01

/* BSE -> pin 151 (aka adc 5V 2)*/
#define IO_PIN_BSE IO_ADC_5V_02

/* RTD -> pin 263 (aka digital in 0) */
#define IO_PIN_RTD IO_DI_00

/* BSE treshold for triggering RTD */
#define BSE_THRESHOLD_RTD 1

/**************************************************************************
 * RTD Settings
 ***************************************************************************/
#define RTD_ON 0
#define RTD_OFF 1

// 1.5 s RTD sound time
#define RTD_SOUND_DURATION MsToUs(1500ul)

/**************************************************************************
 * Controls Settings
 ***************************************************************************/

/* min. difference between the APPS pct travels
 * that'll trigger the implausibility check */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 10

#define IMPLAUSIBILITY_PERSISTENCE_PERIOD_US MsToUs(100ul)

#define MAX_TRAVEL 100

#define MIN_TRAVEL 0


/**************************************************************************
 * Controls Math Macros
 ***************************************************************************/

/* TODO: voltage->pct travel code*/
#define VoltageToPctTravelApps1(val) (0)
#define VoltageToPctTravelApps2(val) (0)

/* TODO: Put in Torque Curve */
#define VoltageToTorque(v) (0)


/**************************************************************************
 * CAN Constants
 ***************************************************************************/

#define CAN_CHANNEL IO_CAN_CHANNEL_0

#define BAUD_RATE 500

/* The system sends CAN messages from a FIFO buffer, this constant defines
 * its size. If a message is added to the full buffer, it is ignored.
 */
#define FIFO_BUFFER_SIZE 20

#define VCU_CONTROLS_CAN_ID 0xC0
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

void main (void)
{
    ubyte4 timestamp;
    ubyte4 implausibility_timestamp;

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

    /* CAN variables and initialization */
    ubyte1 handle_fifo_w;

    IO_CAN_DATA_FRAME controls_can_frame;
    controls_can_frame.id = VCU_CONTROLS_CAN_ID;
    controls_can_frame.id_format = IO_CAN_STD_FRAME;
    controls_can_frame.length = 8;

    IO_CAN_DATA_FRAME debug_can_frame;
    debug_can_frame.id = VCU_DEBUG_CAN_ID;
    debug_can_frame.id_format = IO_CAN_STD_FRAME;
    debug_can_frame.length = 8;
    for (int i = 0; i < 8; i++) {
        debug_can_frame.data[i] = 0;
    }

    IO_CAN_Init( CAN_CHANNEL
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

    /* rtd with 10k pull-up resistor*/
    bool rtd_val;
    IO_DI_Init( IO_PIN_RTD,
                IO_DI_PU_10K );


    /* rtd variables */
    ubyte4 rtd_timestamp;
    bool just_entered_sound_state = TRUE;

    /* VCU State */
    enum VCU_State current_state = NOT_READY;

    // sensor input is invalid during the first cycle
    bool first_cycle = TRUE;


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

                // transitions
                // TODO: Add brake checking
                if (rtd_val == RTD_ON) {
                    // rtd on -> switch to playing rtd sound
                    current_state = PLAYING_RTD_SOUND;
                } else {
                    // if you don't transition away from this state, send a 0
                    // torque message to the motor
                    for (int i = 0; i < 8; i++) {
                        controls_can_frame.data[i] = 0;
                    }
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                }

            } else if (current_state == PLAYING_RTD_SOUND) {

                if (just_entered_sound_state) {
                    // when this state has just been entered, turn on the buzzer and a timer
                    IO_RTC_StartTime(&rtd_timestamp);
                    just_entered_sound_state = FALSE;
                    // TODO: turn on buzzer
                } else if (!just_entered_sound_state && IO_RTC_GetTimeUS(rtd_timestamp) > RTD_SOUND_DURATION) {
                    // once the timer runs out, enter the new state
                    current_state = DRIVING;
                    just_entered_sound_state = TRUE;
                    // TODO: turn off buzzer
                }

                // keep sending 0 torque messages to the inverter in this state
                for (int i = 0; i < 8; i++) {
                    controls_can_frame.data[i] = 0;
                }
                IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);

            } else if (current_state == DRIVING) {
                get_rtd(&rtd_val);

                // transitions
                if (rtd_val == RTD_OFF) {
                    // rtd off -> switch to not ready state
                    current_state = NOT_READY;
                } else {
                    // no transition into another state -> send controls message
                    for (int i = 0; i < 8; i++) {
                        // set everything to 5 for now for debugging purposes
                        controls_can_frame.data[i] = 5;
                    }
                    IO_CAN_WriteFIFO(handle_fifo_w, &controls_can_frame, 1);
                }
            }

            // send out a debug frame with the current state
            debug_can_frame.data[0] = current_state;

            IO_CAN_WriteFIFO(handle_fifo_w, &debug_can_frame, 1);
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
