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
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "APDB.h"

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

/* smallest difference between the 2 APPS pct travels that will
 * trigger the implausibility check
 */
#define APPS_MIN_IMPLAUSIBLE_DEVIATION 10

int voltage_to_pct_travel_apps_1 (ubyte2 val) {
	/*
	 * TODO: write voltage -> pct travel code
	 */
	return 0;
}

int voltage_to_pct_travel_apps_2 (ubyte2 val) {
	/*
	 * TODO: write voltage -> pct travel code
	 */
	return 0;
}

/* absolute value function (so we don't have to import all of stdlib.h
 * for one function
 */
int abs (int k) {
	if (k < 0) {
		return -1 * k;
	}

	return k;
}

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

    int pct_travel_apps_1 = 0;
    int pct_travel_apps_2 = 0;

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

        /*
         *  Application Code
         */

            /* Add your application code here               */
            /*  - IO driver task functions can be used,     */
            /*    to read/write from/to interfaces and IOs  */

        if (!ready_to_drive) {
        	/* retrieve values for RTD switch and BSE */
        	IO_DI_Get(IO_PIN_RTD, &di_val_rtd);
        	IO_ADC_Get(IO_PIN_BSE, &adc_val_bse, &adc_fresh_bse);

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

                pct_travel_apps_1 = voltage_to_pct_travel_apps_1(adc_val_apps_1);
                pct_travel_apps_2 = voltage_to_pct_travel_apps_2(adc_val_apps_2);

                int diff = pct_travel_apps_2 - pct_travel_apps_1;
                diff = abs(diff);

                if (diff >= APPS_MIN_IMPLAUSIBLE_DEVIATION) {
                	/*
                	 * TODO: Implausiblity code
                	 */
                } else {
                	/*
                	 * TODO: send Torque over CAN
                	 */
                }

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
         * 10000 (milliseconds?), this code delays the end of the cycle so it lasts
         * 10000
         */
        while (IO_RTC_GetTimeUS(timestamp) < 10000);
    }
}
