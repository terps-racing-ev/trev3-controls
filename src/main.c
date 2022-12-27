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
 * Controls Settings
 ***************************************************************************/

/* min. difference between the APPS pct travels 
 * that'll trigger the implausibility check */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 10

#define IMPLAUSIBILITY_PERSISTENCE_PERIOD_US MsToUs(100ul)


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

#define BAUD_RATE 250

/* The system sends CAN messages from a FIFO buffer, this constant defines
 * its size. If a message is added to the full buffer, it is ignored.
 */
#define FIFO_BUFFER_SIZE 20

#define VCU_CAN_ID 0



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

void main (void)
{
    ubyte4 timestamp;
    ubyte4 implausibility_timestamp;

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

    /* APPS 1 */
    bool adc_fresh_apps_1;
    ubyte2 adc_val_apps_1;
    IO_ADC_ChannelInit( IO_PIN_APPS_1
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    /* APPS 2 */
    bool adc_fresh_apps_2;
    ubyte2 adc_val_apps_2;
    IO_ADC_ChannelInit( IO_PIN_APPS_2
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    /* BSE */
    bool adc_fresh_bse;
    ubyte2 adc_val_bse;
    IO_ADC_ChannelInit( IO_PIN_BSE
                      , IO_ADC_ABSOLUTE
                      , 0
                      , 0
                      , 0
                      , NULL );

    /* RTD */
    bool di_val_rtd;
    IO_DI_Init( IO_PIN_RTD
    		  , IO_DI_PU_10K );

    bool ready_to_drive = FALSE;
    
    /* APPS variables */
    bool implausible_deviation = FALSE;
    int diff = 0;
    int pct_travel_apps_1 = 0;
    int pct_travel_apps_2 = 0;
    int avg_pct_travel = 0;
    int torque_to_send = 0;
    
    /* CAN variables and initialization */
    ubyte1 handle_fifo_w;
    
    IO_CAN_DATA_FRAME can_frame[1] = {{{0}}};
    
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
    				 , VCU_CAN_ID 
    				 , 0);
    

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/

    /* main loop, executed periodically with a
     * defined cycle time (here: 10 ms)
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

        if (!ready_to_drive) {
        	/* retrieve values for RTD switch and BSE */
        	IO_DI_Get(IO_PIN_RTD, &di_val_rtd);
        	IO_ADC_Get(IO_PIN_BSE, &adc_val_bse, &adc_fresh_bse);
        	
        	/* if brakes are pressed and RTD is activated, activate
        	 * ready-to-drive
        	 */
        	if (di_val_rtd == TRUE && adc_val_bse >= BSE_THRESHOLD_RTD) {
        		ready_to_drive = TRUE;
        		
        		/*
        		 * TODO: RTD BUZZER CODE
        		 */
        	}

        } else {
        	/* check if the RTD switch is on */
        	IO_DI_Get(IO_PIN_RTD, &di_val_rtd);

        	if (di_val_rtd == TRUE) {
        		/* get the raw values for both APPS sensors */
                IO_ADC_Get(IO_PIN_APPS_1, &adc_val_apps_1, &adc_fresh_apps_1);
                IO_ADC_Get(IO_PIN_APPS_2, &adc_val_apps_2, &adc_fresh_apps_2);
                
                /* calculate pct travel */
                if (adc_fresh_apps_1 && adc_fresh_apps_2) {
                	pct_travel_apps_1 = VoltageToPctTravelApps1(adc_val_apps_1);
                	pct_travel_apps_2 = VoltageToPctTravelApps2(adc_val_apps_2);
                }
                
                /* get the difference between the two */
                diff = pct_travel_apps_2 - pct_travel_apps_1;
                diff = Abs(diff);
                
                /* if the difference is above the implausibility requirement */
                if (diff >= APPS_MIN_IMPLAUSIBLE_DEVIATION) {
                	IO_RTC_StartTime(&implausibility_timestamp);
                	
                	implausible_deviation = TRUE;
                	
                	/* check if the implausiblity persists for 100 msec */
                	while (IO_RTC_GetTimeUS(implausibility_timestamp) 
                			< IMPLAUSIBILITY_PERSISTENCE_PERIOD_US
                			&& implausible_deviation) {
                		
                		/* keep checking the difference between the 2 apps's */
                        IO_ADC_Get(IO_PIN_APPS_1, &adc_val_apps_1, &adc_fresh_apps_1);
                        IO_ADC_Get(IO_PIN_APPS_2, &adc_val_apps_2, &adc_fresh_apps_2);
                        
                        if (adc_fresh_apps_1 && adc_fresh_apps_2) {
                        	pct_travel_apps_1 = VoltageToPctTravelApps1(adc_val_apps_1);
                        	pct_travel_apps_2 = VoltageToPctTravelApps2(adc_val_apps_2);
                        }
                        
                        diff = pct_travel_apps_2 - pct_travel_apps_1;
                        diff = Abs(diff);
                        
                        /* stop checking once the 2 values don't deviate anymore */
                        if (diff < APPS_MIN_IMPLAUSIBLE_DEVIATION) {
                        	implausible_deviation = FALSE;
                        }
                	}

                }
            	
                /* calculate the torque to be sent */
            	if (implausible_deviation) {
            		torque_to_send = 0;
            	} else {
            		avg_pct_travel = ((pct_travel_apps_2 + pct_travel_apps_1) / 2);
            		torque_to_send = VoltageToTorque(avg_pct_travel);
            	}
            	
            	can_frame[0].id = VCU_CAN_ID;
            	can_frame[0].id_format = IO_CAN_STD_FRAME;
            	can_frame[0].length = 8;
            	can_frame[0].data[0] = (torque_to_send * 10) % 256;
            	can_frame[0].data[1] = (torque_to_send * 10) / 256;
            	can_frame[0].data[2] = 0;
            	can_frame[0].data[3] = 0;
            	can_frame[0].data[4] = 1;
            	can_frame[0].data[5] = 0;
            	can_frame[0].data[6] = 0;
            	can_frame[0].data[7] = 0;
            	
            	IO_CAN_WriteFIFO(handle_fifo_w, can_frame, 1);

        	} else {
        		/* If the RTD switch has been turned off, the RTD procedure
        		 * has to be repeated
        		 */
        		ready_to_drive = FALSE;
        	}
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
        while (IO_RTC_GetTimeUS(timestamp) < MsToUs(10));
    }
}
