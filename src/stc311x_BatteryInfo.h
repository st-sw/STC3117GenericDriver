/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : stc311x_BatteryInfo.h
* Author             : AMS - IMS application
* Version            : V00
* Description        : Application/Battery description
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __Battery_Info_H
#define __Battery_Info_H



/* ******************************************************************************** */
/*        INTERNAL PARAMETERS                                                       */
/*   TO BE ADJUSTED ACCORDING TO BATTERY/APPLICATION CHARACTERISTICS                */
/* -------------------------------------------------------------------------------- */
/*                                                                                  */

/*Battery parameters define  ------------------------------------------------------ */
#define BATT_CAPACITY		1650	/* battery nominal capacity in mAh					*/
#define BATT_RINT			200		/* Internal battery impedance in mOhms, 0 if unknown	*/

/*Application parameters define  -------------------------------------------------- */
#define RSENSE				10		/* current sensing resistor (soldered on the board) in mOhms */

#define BATT_CHG_VOLTAGE   4250   /* min voltage at the end of the charge (mV)      */
#define BATT_MIN_VOLTAGE   3300   /* nearly empty battery detection level (mV)      */
#define MAX_HRSOC          51200  /* HRSOC (Higher Resolution SOC): 100% in 1/512% units */
#define MAX_SOC            1000   /* 100% in 0.1% units */

#define CHG_MIN_CURRENT     150   /* min charge current in mA                       */
#define CHG_END_CURRENT      20   /* end charge current in mA                       */
#define APP_MIN_CURRENT     (-5)  /* minimum application current consumption in mA ( <0 !) */
#define APP_MIN_VOLTAGE	    3000  /* application cut-off voltage                    */
#define TEMP_MIN_ADJ	    (-5)  /* minimum temperature for gain adjustment */

#define VMTEMPTABLE        { 85, 90, 100, 160, 320, 440, 840 }  /* normalized VM_CNF at 60, 40, 25, 10, 0, -10°C, -20°C */

#define AVGFILTER           4  /* average filter constant */


/* Define the default battery OCV curve to be used for initialization:  */
#define DEFAULT_BATTERY_4V20_MAX      //Default OCV curve for a 4.20V max battery
//#define DEFAULT_BATTERY_4V35_MAX      //Default OCV curve for a 4.35V max battery
//#define CUSTOM_BATTERY_OCV          //OCV curve determined from battery manufacturer data, or battery characterization statistics.

#define MONITORING_MODE   MIXED_MODE  /* 1=Voltage mode, 0=mixed mode */

/* ******************************************************************************** */


#endif

/**** END OF FILE ****/
