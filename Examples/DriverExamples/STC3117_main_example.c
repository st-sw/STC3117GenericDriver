/******************** (C) COPYRIGHT STMicroelectronics ********************
* File Name          : MainExample.c
* Author             : AMS application
* Version            : V1.02
* Date               : 2016/09/01
* Description        : STC3117 gas gauge driver init example
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

#include "stc311x_gasgauge.h"
#include <stdio.h>
#include "stc311x_BatteryInfo.h"

static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct);
static int GasGaugeTimerFinished(void);
static void Delay_ms(unsigned int value);

#define TIMER_LIMIT 0x5000 //arbitrary value to modify, and to wait 5s in normal use case
#define SYSTEM_TIMER_AVAILABLE

int main(void)
{
	GasGauge_DataTypeDef STC3117_GG_struct;
	int Voltage;
	int Soc;
	int Current;
	int status;
	int CounterValue;
	int i;
	int BatteryMonitoringEnabled = 1; //true


	//Optional: HardwareShutDown set to 1 when the user power down the hardware, and no need to monitor the battery. 
	//But not the recommanded case, as it is better to have the gas gauge always running for better accuracy, even if the rest of the system is powered down
	volatile char HardwareShutDown = 0; 


GasGauge_Restart:

	printf("STC3117 fuel gauge driver init ...\n");


	//----------------------------------------------------------------------
	//Check I2C is working and Fuel gauge device connected
	status = STC31xx_CheckI2cDeviceId();
	if (status != 0) //error
	{
		if(status == -1)
		{
			printf("STC3117: I2C error\n");
#ifdef DEBUG
			//wait to simulate the whole application restart
			while( GasGaugeTimerFinished() != 1);
			goto GasGauge_Restart;
#endif
		}
		else if(status == -2)
			printf("STC3117: Wrong device detected\n");
		else
			printf("STC3117: Unknown Hardware error\n");

		return -1; //return on error
	}


	//----------------------------------------------------------------------
	// Check Gasgauge is powered up & ready, 
	// and wait first battery measurement (V, I) is done (i.e. wait CounterValue is 3 or more)

#ifdef SYSTEM_TIMER_AVAILABLE //wait implementation with timeout

	for(i=0; i<20; i++)  //check for 20*100ms = 2s
	{
		CounterValue = STC31xx_GetRunningCounter();
		if(CounterValue >= 3) //ok, device ready
		{
			break; //exit loop
		}
		else if(CounterValue < 0) //communication Error
		{
			printf("STC3117: Error at power up.\n");
			goto GasGauge_Restart;
		}
		else
		{
			//wait for battery measurement
			Delay_ms(100);
		}
	}

	if(CounterValue < 3) //timeout, the devise has not started
	{
		printf("STC3117 timeout: Error at power up.\n");
		goto GasGauge_Restart;
	}

#else //simple implementation but without timeout checked (can be stucked)

	CounterValue = 0;
	while(CounterValue < 3) //wait till device ready
	{
		CounterValue = STC31xx_GetRunningCounter();

		if(CounterValue < 0) //communication Error
		{
			printf("STC3117: Error at power up.\n");
			goto GasGauge_Restart;
		}
	}
#endif



	//----------------------------------------------------------------------
	//Call STC3117 driver init&start function

	//Config init
	GasGauge_DefaultInit(&STC3117_GG_struct);

	//Call STC311x driver START initialization function
	status = GasGauge_Start(&STC3117_GG_struct);

	if(status!=0 && status!=-2)
	{
		printf("Error in GasGauge_Start\n");
		return -1; //return on error
	}


	while(BatteryMonitoringEnabled) //main infinite loop
	{
		if(HardwareShutDown == 1) //var modified from IRQ routine
		{
			HardwareShutDown = 0; //clear

			status = GasGauge_Stop();
			if(status != 0) printf("Error in GasGauge_Stop\n");

			BatteryMonitoringEnabled = 0; //end of monitoring
		}
		else //normal case
		{
			if(GasGaugeTimerFinished() == 1) //Process task every 5s (or every 1s to 30s)
			{
				//Call task function	
				status = GasGauge_Task(&STC3117_GG_struct);  /* process gas gauge algorithm, returns results */

				if (status > 0) //OK, new data available
				{
					/* results available */
					Soc = STC3117_GG_struct.SOC;
					Voltage = STC3117_GG_struct.Voltage;
					Current = STC3117_GG_struct.Current;

					printf("Vbat: %i mV, I=%i mA SoC=%i, C=%i \r\n", 
						STC3117_GG_struct.Voltage, 
						STC3117_GG_struct.Current, 
						STC3117_GG_struct.SOC, 
						STC3117_GG_struct.ChargeValue);
				}
				else if(status == 0) //only previous SOC, OCV and voltage are valid
				{
					printf("Previous_SoC=%i, OCV=%i \r\n", 
						STC3117_GG_struct.SOC, 
						STC3117_GG_struct.OCV);
				}
				else if(status == -1) //error occurred
				{
					/* results available */
					//Soc = (STC3117_GG_struct.SOC+5)/10;
					//Voltage = STC3117_GG_struct.Voltage;
				}
			}
			else
			{
				//Do other Tasks here ...

				//printf("Waiting next battery measurement...\r\n");
			}
		}
	}

	return 0;
}


static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct)
{
	int Rint;


	//structure initialisation

	GG_struct->Cnom = BATT_CAPACITY;        /* nominal Battery capacity in mAh */  //Warning: Battery dependant. Put the corresponding used value.
	
	GG_struct->Vmode = MONITORING_MODE;       /* 1=Voltage mode, 0=mixed mode */
	

	Rint = BATT_RINT;
	if (Rint == 0)  Rint = 200; //force default

	GG_struct->VM_cnf = (int) ((Rint * BATT_CAPACITY) / 977.78);       /* nominal VM cnf */


	if (MONITORING_MODE == MIXED_MODE)
	{
		GG_struct->CC_cnf = (int)((RSENSE * BATT_CAPACITY) / 49.556);     /* nominal CC_cnf, for CC mode only */
	}


	GG_struct->SoctabValue[0] = 0;        /* SOC curve adjustment = 0%*/
	GG_struct->SoctabValue[1] = 3 * 2;    /* SOC curve adjustment = 3%*/
	GG_struct->SoctabValue[2] = 6 * 2;    /* SOC curve adjustment = 6%*/
	GG_struct->SoctabValue[3] = 10 * 2;    /* SOC curve adjustment = 10%*/
	GG_struct->SoctabValue[4] = 15 * 2;    /* SOC curve adjustment = 15%*/
	GG_struct->SoctabValue[5] = 20 * 2;    /* SOC curve adjustment = 20%*/
	GG_struct->SoctabValue[6] = 25 * 2;    /* SOC curve adjustment = 25%*/
	GG_struct->SoctabValue[7] = 30 * 2;    /* SOC curve adjustment = 30%*/
	GG_struct->SoctabValue[8] = 40 * 2;    /* SOC curve adjustment = 40%*/
	GG_struct->SoctabValue[9] = 50 * 2;    /* SOC curve adjustment = 50%*/
	GG_struct->SoctabValue[10] = 60 * 2;    /* SOC curve adjustment = 60%*/
	GG_struct->SoctabValue[11] = 65 * 2;    /* SOC curve adjustment = 65%*/
	GG_struct->SoctabValue[12] = 70 * 2;    /* SOC curve adjustment = 70%*/
	GG_struct->SoctabValue[13] = 80 * 2;    /* SOC curve adjustment = 80%*/
	GG_struct->SoctabValue[14] = 90 * 2;    /* SOC curve adjustment = 90%*/
	GG_struct->SoctabValue[15] = 100 * 2;    /* SOC curve adjustment = 100%*/


#ifdef	DEFAULT_BATTERY_4V20_MAX      //Default OCV curve for a 4.20V max battery
	GG_struct->OcvValue[0] = 0x1770;    /* OCV curve value at 0%*/
	GG_struct->OcvValue[1] = 0x1926;    /* OCV curve value at 3%*/
	GG_struct->OcvValue[2] = 0x19B2;    /* OCV curve value at 6%*/
	GG_struct->OcvValue[3] = 0x19FB;    /* OCV curve value at 10%*/
	GG_struct->OcvValue[4] = 0x1A3E;    /* OCV curve value at 15%*/
	GG_struct->OcvValue[5] = 0x1A6D;    /* OCV curve value at 20%*/
	GG_struct->OcvValue[6] = 0x1A9D;    /* OCV curve value at 25%*/
	GG_struct->OcvValue[7] = 0x1AB6;    /* OCV curve value at 30%*/
	GG_struct->OcvValue[8] = 0x1AD5;    /* OCV curve value at 40%*/
	GG_struct->OcvValue[9] = 0x1B01;    /* OCV curve value at 50%*/
	GG_struct->OcvValue[10] = 0x1B70;    /* OCV curve value at 60%*/
	GG_struct->OcvValue[11] = 0x1BB1;    /* OCV curve value at 65%*/
	GG_struct->OcvValue[12] = 0x1BE8;    /* OCV curve value at 70%*/
	GG_struct->OcvValue[13] = 0x1C58;    /* OCV curve value at 80%*/
	GG_struct->OcvValue[14] = 0x1CF3;    /* OCV curve value at 90%*/
	GG_struct->OcvValue[15] = 0x1DA9;    /* OCV curve value at 100%*/
#endif

#ifdef	DEFAULT_BATTERY_4V35_MAX      //Default OCV curve for a 4.35V max battery
	GG_struct->OcvValue[0] = 0x1770;    /* OCV curve value at 0%*/
	GG_struct->OcvValue[1] = 0x195D;    /* OCV curve value at 3%*/
	GG_struct->OcvValue[2] = 0x19EE;    /* OCV curve value at 6%*/
	GG_struct->OcvValue[3] = 0x1A1A;    /* OCV curve value at 10%*/
	GG_struct->OcvValue[4] = 0x1A59;    /* OCV curve value at 15%*/
	GG_struct->OcvValue[5] = 0x1A95;    /* OCV curve value at 20%*/
	GG_struct->OcvValue[6] = 0x1AB6;    /* OCV curve value at 25%*/
	GG_struct->OcvValue[7] = 0x1AC7;    /* OCV curve value at 30%*/
	GG_struct->OcvValue[8] = 0x1AEB;    /* OCV curve value at 40%*/
	GG_struct->OcvValue[9] = 0x1B2B;    /* OCV curve value at 50%*/
	GG_struct->OcvValue[10] = 0x1BCC;    /* OCV curve value at 60%*/
	GG_struct->OcvValue[11] = 0x1C13;    /* OCV curve value at 65%*/
	GG_struct->OcvValue[12] = 0x1C57;    /* OCV curve value at 70%*/
	GG_struct->OcvValue[13] = 0x1D09;    /* OCV curve value at 80%*/
	GG_struct->OcvValue[14] = 0x1DCF;    /* OCV curve value at 90%*/
	GG_struct->OcvValue[15] = 0x1EA2;    /* OCV curve value at 100%*/
#endif

#ifdef	CUSTOM_BATTERY_OCV   //fill custom battery data
	GG_struct->OcvValue[0] = ...;
	GG_struct->OcvValue[1] = ...;
	//...
	//...
#endif

	GG_struct->CapDerating[6]=0;   /* capacity derating in 0.1%, for temp = -20 �C */
	GG_struct->CapDerating[5]=0;   /* capacity derating in 0.1%, for temp = -10 �C */
	GG_struct->CapDerating[4]=0;   /* capacity derating in 0.1%, for temp = 0   �C */
	GG_struct->CapDerating[3]=0;   /* capacity derating in 0.1%, for temp = 10  �C */
	GG_struct->CapDerating[2]=0;   /* capacity derating in 0.1%, for temp = 25  �C */
	GG_struct->CapDerating[1]=0;   /* capacity derating in 0.1%, for temp = 40  �C */
	GG_struct->CapDerating[0]=0;   /* capacity derating in 0.1%, for temp = 60  �C */


	

	GG_struct->Alm_SOC = 10;     /* SOC alm level % */
	GG_struct->Alm_Vbat = 3600;    /* Vbat alm level mV */


	GG_struct->Rsense = RSENSE;      /* sense resistor mOhms */   //Warning: Hardware dependant. Put the corresponding used value
	GG_struct->RelaxCurrent = GG_struct->Cnom/20;  /* current for relaxation (< C/20) mA */

	GG_struct->ForceExternalTemperature = 0; //0: do not force Temperature but Gas gauge measures it

}



static int GasGaugeTimerFinished(void)
{
	static unsigned int i = 1;

	i--;
	if( i == 0)
	{
		i = TIMER_LIMIT;
		return 1;
	}
	else
	{
		return 0;
	}
}



#ifdef SYSTEM_TIMER_AVAILABLE

static void Delay_ms(unsigned int value) 
{
	//quick and dirty delay function implementation. Use a real timer instead.

	unsigned int i,j;
	volatile int dummy = 0;

	for(i=0; i<value; i++)
	{
		for(j=0; j<2000; j++) //pseudo 1ms delay
		{
			// waste function, volatile makes sure it is not being optimized out by compiler
			dummy++;
		}
	}
}

#endif

