/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : stc311x_gasgauge.c
* Author             : AMS application
* Version            : V2.08
* Date               : 2016/09/01
* Description        : gas gauge firmware for STC3117
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

/* Includes ------------------------------------------------------------------*/
#include "stc311x_gasgauge.h" 
#include "Generic_I2C.h"

/*Function declaration*/
int STC31xx_SetPowerSavingMode(void);
int STC31xx_StopPowerSavingMode(void);
int STC31xx_AlarmSet(void);
int STC31xx_AlarmStop(void);
int STC31xx_AlarmGet(void);
int STC31xx_AlarmClear(void);
int STC31xx_AlarmSetVoltageThreshold(int VoltThresh);
int STC31xx_AlarmSetSOCThreshold(int SOCThresh);
int STC31xx_RelaxTmrSet(int CurrentThreshold);



/* ******************************************************************************** */
/*        STC311x DEVICE SELECTION                                                  */
/* -------------------------------------------------------------------------------- */
//#define STC3115
#define STC3117
/* ******************************************************************************** */



/* Private define ------------------------------------------------------------*/


/* ******************************************************************************** */
/*        SPECIAL FUNCTIONS                                                         */
/* -------------------------------------------------------------------------------- */
/*                                                                                  */
/* define TEMP_COMPENSATION_SOC to enable SOC temperature compensation */
#define TEMP_COMPENSATION_SOC

#define OCV_RAM_BACKUP

//#define BATD_UC8 //Optional: basic reset of the gas gauge in case of error event occurs (BATD or UVLO)

/* ******************************************************************************** */


/* ******************************************************************************** */
/*        INTERNAL PARAMETERS                                                       */
/*   TO BE ADJUSTED ACCORDING TO BATTERY/APPLICATION CHARACTERISTICS                */
/* -------------------------------------------------------------------------------- */
/*                                                                                  */
#define BATT_CHG_VOLTAGE   4250   /* min voltage at the end of the charge (mV)      */
#define BATT_MIN_VOLTAGE   3300   /* nearly empty battery detection level (mV)      */
#define MAX_HRSOC          51200  /* HRSOC (Higher Resolution SOC): 100% in 1/512% units */
#define MAX_SOC            1000   /* 100% in 0.1% units */
/*                                                                                  */
#define CHG_MIN_CURRENT     150   /* min charge current in mA                       */
#define CHG_END_CURRENT      20   /* end charge current in mA                       */
#define APP_MIN_CURRENT     (-5)  /* minimum application current consumption in mA ( <0 !) */
#define APP_MIN_VOLTAGE	    3000  /* application cut-off voltage                    */
#define TEMP_MIN_ADJ	    (-5)  /* minimum temperature for gain adjustment */

#define VMTEMPTABLE        { 85, 90, 100, 160, 320, 440, 840 }  /* normalized VM_CNF at 60, 40, 25, 10, 0, -10°C, -20°C */

#define AVGFILTER           4  /* average filter constant */

/* ******************************************************************************** */



/* Private define ------------------------------------------------------------*/

#define STC31xx_SLAVE_ADDRESS            0xE0   /* STC31xx 8-bit address byte */

/*Address of the STC311x register --------------------------------------------*/
#define STC311x_REG_MODE                 0x00    /* Mode Register             */
#define STC311x_REG_CTRL                 0x01    /* Control and Status Register */
#define STC311x_REG_SOC                  0x02    /* SOC Data (2 bytes) */
#define STC311x_REG_COUNTER              0x04    /* Number of Conversion (2 bytes) */
#define STC311x_REG_CURRENT              0x06    /* Battery Current (2 bytes) */
#define STC311x_REG_VOLTAGE              0x08    /* Battery Voltage (2 bytes) */
#define STC311x_REG_TEMPERATURE          0x0A    /* Temperature               */
#ifdef STC3117
#define STC311x_REG_AVG_CURRENT          0x0B    /* Battery Average Current (2 bytes)   */
#endif
#define STC311x_REG_OCV                  0x0D    /* Battery OCV (2 bytes) */
#define STC311x_REG_CC_CNF               0x0F    /* CC configuration (2 bytes)    */
#define STC311x_REG_VM_CNF               0x11    /* VM configuration (2 bytes)    */
#define STC311x_REG_ALARM_SOC            0x13    /* SOC alarm level         */
#define STC311x_REG_ALARM_VOLTAGE        0x14    /* Low voltage alarm level */
#define STC311x_REG_CURRENT_THRES        0x15    /* Current threshold for relaxation */
#define STC311x_REG_CMONIT_COUNT         0x16    /* Current monitoring counter   */
#define STC311x_REG_CMONIT_MAX           0x17    /* Current monitoring max count */

#ifdef STC3117
#define STC311x_REG_CC_ADJ               0x1B    /* CC adjustement (2 bytes)    */
#define STC311x_REG_VM_ADJ               0x1D    /* VM adjustement (2 bytes)    */
#endif

#ifdef STC3115
#define STC311x_REG_CC_ADJ_HIGH          0x0B    /* CC adjustement     */
#define STC311x_REG_VM_ADJ_HIGH          0x0C    /* VM adjustement     */
#define STC311x_REG_CC_ADJ_LOW           0x19    /* CC adjustement     */
#define STC311x_REG_VM_ADJ_LOW           0x1A    /* VM adjustement     */
#define STC311x_ACC_CC_ADJ_HIGH          0x1B    /* CC accumulator     */
#define STC311x_ACC_CC_ADJ_LOW           0x1C    /* CC accumulator     */
#define STC311x_ACC_VM_ADJ_HIGH          0x1D    /* VM accumulator     */
#define STC311x_ACC_VM_ADJ_LOW           0x1E    /* VM accumulator     */
#endif


/*Bit mask definition*/
#define STC311x_VMODE   		 0x01	 /* Voltage mode bit mask     */
#define STC311x_ALM_ENA			 0x08	 /* Alarm enable bit mask     */
#define STC311x_GG_RUN			 0x10	 /* Alarm enable bit mask     */
#define STC311x_FORCE_CC		 0x20	 /* Force CC bit mask     */
#define STC311x_FORCE_VM		 0x40	 /* Force VM bit mask     */
#define STC311x_SOFTPOR 		 0x11	 /* soft reset     */
#ifdef STC3115
#define STC311x_CLR_VM_ADJ               0x02  /* Clear VM ADJ register bit mask */
#define STC311x_CLR_CC_ADJ               0x04  /* Clear CC ADJ register bit mask */
#endif
#ifdef STC3117
#define STC311x_BATD_PU                  0x02  /* Enable internal Pull-Up on BATD bit mask */
#define STC311x_FORCE_CD                 0x04  /* Force CD high bit mask */
#endif

#define STC311x_REG_ID                   0x18    /* Chip ID addr (1 byte)       */
#define STC3115_ID                       0x14    /* STC3115 ID */
#define STC3117_ID                       0x16    /* STC3117 ID */

#define STC311x_REG_RAM                  0x20    /* General Purpose RAM Registers */
#define RAM_SIZE                         16      /* Total RAM size of STC311x in bytes */

#define STC311x_REG_OCVTAB               0x30
#define OCVTAB_SIZE                      16      /* OCVTAB size of STC311x */
#define OCVTAB_BYTECOUNT                 2

#define STC311x_REG_SOCTAB               0x50
#define SOCTAB_SIZE                      16      /* SOCTAB size of STC311x */


#ifdef STC3115
#define VCOUNT				 4       /* counter value for 1st current/temp measurements */
#endif
#ifdef STC3117
#define VCOUNT				 0       /* counter value for 1st current/temp measurements */
#endif



#define M_STATUS_MSK 0x1010       /* GG_RUN & PORDET mask in STC311x_BattDataTypeDef status word */
#define M_RST_ERR_MSK 0x1800       /* BATFAIL & PORDET mask */
#define M_RUN_MSK  0x0010       /* GG_RUN mask in STC311x_BattDataTypeDef status word */
#define M_GGVM_MSK 0x0400       /* GG_VM mask */
#define M_BATFAIL_MSK 0x0800    /* BATFAIL mask*/
#ifdef STC3117
#define M_UVLOD_MSK   0x8000    /* UVLOD mask (STC3117 only) */
#endif
#define M_VMOD_MSK 0x0001       /* VMODE mask */

#define STC31XX_OK 0

/* Battery charge state definition for BattState */
#define  BATT_CHARGING  3
#define  BATT_ENDCHARG  2
#define  BATT_FULCHARG  1
#define  BATT_IDLE      0
#define  BATT_DISCHARG (-1)
#define  BATT_LOWBATT  (-2)

/* STC311x RAM test word */
#define RAM_TESTWORD 0x53A9

/* Gas gauge states */
#define GG_INIT     'I'
#define GG_RUNNING  'R'
#define GG_POWERDN  'D'

#define VM_MODE 1  // Voltage mode
#define CC_MODE 0  // Mixed mode



/* gas gauge structure definition ------------------------------------*/

/* Private constants ---------------------------------------------------------*/

#define NTEMP 7
static const int TempTable[NTEMP] = {60, 40, 25, 10, 0, -10, -20} ;   /* temperature table from 60°C to -20°C (descending order!) */
static const int DefVMTempTable[NTEMP] = VMTEMPTABLE;

/* Private variables ---------------------------------------------------------*/

/* structure of the STC311x battery monitoring data */
typedef struct  {
	/* STC311x data */
	int STC_Status;  /* status word  */
	int Vmode;       /* 1=Voltage mode, 0=mixed mode */
	int Voltage;     /* voltage in mV            */
	int Current;     /* current in mA            */
	int Temperature; /* temperature in 0.1°C     */
	int HRSOC;       /* High Resolution uncompensated SOC in 1/512%   */
	int OCV;         /* OCV in mV*/
	int ConvCounter; /* convertion counter       */
	int RelaxTimer;  /* current relax timer value */
	int CC_adj;      /* CC adj */
	int VM_adj;      /* VM adj */
	/* results & internals */
	int SOC;         /* compensated SOC in 0.1% */
	int AvgSOC;      /* in 0.1% */
	int AvgVoltage;
	int AvgCurrent;
	int AvgTemperature;
	int AccSOC;
	int AccVoltage;
	int AccCurrent;
	int AccTemperature;
	int BattState;
	int GG_Mode;     /* 1=VM active, 0=CC active */
	int LastTemperature;
	int BattOnline;	// BATD
	int IDCode;
	/* parameters */
	int Alm_SOC;     /* SOC alm level in % */
	int Alm_Vbat;    /* Vbat alm level in mV */
	int CC_cnf;      /* nominal CC_cnf */
	int VM_cnf;      /* nominal VM cnf */
	int Cnom;        /* nominal capacity is mAh */
	int Rsense;      /* sense resistor in milliOhms */
	int Rint;        /* internal resistance in milliOhms */
	int CurrentFactor;
	int CRateFactor;
	int RelaxThreshold;   /* current threshold for VM (mA)  */
	int VM_TempTable[NTEMP];
	int CapacityDerating[NTEMP];
#ifdef STC3115
	char OCVOffset[OCVTAB_SIZE];
#endif
#ifdef STC3117  
	int  OcvValue[OCVTAB_SIZE];
	unsigned char SoctabValue[SOCTAB_SIZE];
#endif 
	int  Ropt;
	int  Nropt;

} STC311x_BattDataTypeDef;

static STC311x_BattDataTypeDef BattData;   /* STC311x data */

/* structure of the STC311x RAM registers for the Gas Gauge algorithm data */
static union {
	unsigned char db[RAM_SIZE];  /* last byte holds the CRC */
	struct {
		short int TestWord;     /* 0-1 Test Word for verification */
		short int HRSOC;       /* 2-3 SOC backup */
		short int CC_cnf;      /* 4-5 current CC_cnf */
		short int VM_cnf;      /* 6-7 current VM_cnf */
		char SOC;              /* 8 SOC for trace (in %) */
		char GG_Status;        /* 9  */
		/* bytes ..RAM_SIZE-2 are free, last byte RAM_SIZE-1 is the CRC */
	} reg;
} GG_Ram;




/* -----------------------------------------------------------------
The following routines interface with the I2C primitives 
I2C_ReadBytes(u8_I2C_address, u8_NumberOfBytes, u8_RegAddress, pu8_RxBuffer);
I2C_WriteBytes(u8_I2C_address, u8_NumberOfBytes, u8_RegAddress, pu8_TxBuffer);
note: here I2C_Address is the 8-bit address byte
----------------------------------------------------------------- */

#define NBRETRY 3  //I2C communication retry in case of Bus conflict


/*******************************************************************************
* Function Name  : STC31xx_Write
* Description    : utility function to write several bytes to STC311x registers
* Input          : NumberOfBytes, RegAddress, TxBuffer
* Return         : error status
* Note: Recommended implementation is to used I2C block write. If not available,
* STC311x registers can be written by 2-byte words (unless NumberOfBytes=1)
* or byte per byte.
*******************************************************************************/
static int STC31xx_Write(int NumberOfBytes, int RegAddress , unsigned char *TxBuffer)
{
	int retry;
	int res=-1;

	for (retry=0; retry < NBRETRY; retry++)
	{      
		res = I2C_WriteBytes(STC31xx_SLAVE_ADDRESS, RegAddress, TxBuffer, NumberOfBytes);
		if (res==STC31XX_OK) break;
	}
	return(res);
}



/*******************************************************************************
* Function Name  : STC31xx_Read
* Description    : utility function to read several bytes from STC311x registers
* Input          : NumberOfBytes, RegAddress, , RxBuffer
* Return         : error status
* Note: Recommended implementation is to used I2C block read. If not available,
* STC311x registers can be read by 2-byte words (unless NumberOfBytes=1)
* Using byte per byte read is not recommended since it doesn't ensure register data integrity
*******************************************************************************/
static int STC31xx_Read(int NumberOfBytes, int RegAddress , unsigned char *RxBuffer)
{
	int retry;
	int res = -1;

	for (retry=0; retry < NBRETRY; retry++)
	{
		res = I2C_ReadBytes(STC31xx_SLAVE_ADDRESS, RegAddress, RxBuffer, NumberOfBytes);
		if (res == STC31XX_OK) break;
	}
	return(res);    
}



/* ---- end of I2C primitive interface --------------------------------------------- */


/*******************************************************************************
* Function Name  : STC31xx_ReadByte8
* Description    : utility function to read the value stored in one register
* Input          : RegAddress: STC311x register,
* Return         : 8-bit value, or 0 if error
*******************************************************************************/
static int STC31xx_ReadByte8(int RegAddress)
{
	int value;
	unsigned char data[2];
	int res;

	res=STC31xx_Read(1, RegAddress, data);

	if (res >= 0)
	{
		/* no error */
		value = data[0];
	}
	else
		value=0;

	return(value);
}



/*******************************************************************************
* Function Name  : STC31xx_WriteByte8
* Description    : utility function to write a 8-bit value into a register
* Input          : RegAddress: STC311x register, Value: 8-bit value to write
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC31xx_WriteByte8(int RegAddress, unsigned char Value)
{
	int res;
	unsigned char data[2];

	data[0]= Value; 
	res = STC31xx_Write(1, RegAddress, data);

	return(res);

}


/*******************************************************************************
* Function Name  : STC31xx_ReadWord16
* Description    : utility function to read the value stored in one register pair
* Input          : RegAddress: STC311x register,
* Return         : 16-bit value, or 0 if error
*******************************************************************************/
static int STC31xx_ReadWord16(int RegAddress)
{
	int value;
	unsigned char data[2];
	int res;

	res=STC31xx_Read(2, RegAddress, data);

	if (res >= 0)
	{
		/* no error */
		value = data[1];
		value = (value <<8) + data[0];
	}
	else
		value=0;

	return(value);
}


int STC31xx_ReadUnsignedWord16(unsigned short RegAddress, unsigned short * RegData)
{
	unsigned short data16;
	unsigned char data8[2];
	int status;

	status = STC31xx_Read(2, RegAddress , data8);

	if (status >= 0)
	{
		/* no error */
		data16 = data8[1];
		data16 = (data16 <<8) | data8[0];

		*RegData = data16;
	}
	else
		status = -1;

	return(status);
}


/*******************************************************************************
* Function Name  : STC31xx_WriteWord16
* Description    : utility function to write a 16-bit value into a register pair
* Input          : RegAddress: STC311x register, Value: 16-bit value to write
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC31xx_WriteWord16(int RegAddress, int Value)
{
	int res;
	unsigned char data[2];

	data[0]= Value & 0xff; 
	data[1]= (Value>>8) & 0xff; 
	res = STC31xx_Write(2, RegAddress, data);

	return(res);

}



/* ---- end of I2C R/W interface --------------------------------------------- */


/* -------------------------------------------------------------------------- */

/* #define CurrentFactor  (24084/SENSERESISTOR)         LSB=5.88uV/R= ~24084/R/4096 - convert to mA  */
#define VoltageFactor  9011                          /* LSB=2.20mV ~9011/4096 - convert to mV         */



/*******************************************************************************
* Function Name  : STC311x_GetStatusWord16
* Description    :  Read the STC311x status
* Input          : None
* Return         : status word (REG_MODE / REG_CTRL), -1 if error
*******************************************************************************/
static int STC311x_GetStatusWord16(void)
{
	int value;

	/* first, check the presence of the STC311x by reading first byte of dev. ID */
	BattData.IDCode = STC31xx_ReadByte8(STC311x_REG_ID);
	if (BattData.IDCode!= STC3115_ID && BattData.IDCode!= STC3117_ID) return (-1);

	/* read REG_MODE and REG_CTRL */
	value = STC31xx_ReadWord16(STC311x_REG_MODE);
	value &= 0x7fff;   //(MSbit is unused, but used for error dectection here)

	return (value);
}



/*******************************************************************************
* Function Name  : STC3117_CheckI2cDeviceId
* Description    :  Read the hardcoded STC3115 ID number
* Input          : pointer to char
* Return         : status, -1 if error, -2 if bad ID
*******************************************************************************/
int STC31xx_CheckI2cDeviceId(void)
{
	unsigned char RegAddress;
	unsigned char data8;
	int status;

	RegAddress = STC311x_REG_ID;
	status = STC31xx_Read(1, RegAddress , &data8);

	if (status >= 0)
	{
		if(data8 == STC3117_ID)
		{
			status = 0; //OK
		}
		else
		{
			status = -2; //I2C is working, but the ID doesn't match.
		}
	}
	else
	{
		status = -1; // I2C error
	}

	return(status);
}

/*******************************************************************************
* Function Name  : STC3117_GetRunningCounter
* Description    :  Get the STC3115 Convertion counter value
* Input          : None
* Return         : status word (REG_COUNTER), -1 if error
*******************************************************************************/
int STC31xx_GetRunningCounter(void)
{
	unsigned short value;
	int status;

	/* read STC3117_REG_COUNTER */
	status = STC31xx_ReadUnsignedWord16(STC311x_REG_COUNTER, &value);

	if(status < 0) //error
		value = -1;

	return ((int)value);
}


/*******************************************************************************
* Function Name  : STC311x_SetInitialParam
* Description    :  initialize the STC311x parameters
* Input          : rst: init algo param
* Return         : 0
*******************************************************************************/
static void STC311x_SetInitialParam(void)
{
	int value;
	int i;

	STC31xx_WriteByte8(STC311x_REG_MODE, 0x01);  /*   set GG_RUN=0 before changing algo parameters */

	/* init OCV curve */
#ifdef STC3115  
	STC31xx_Write(OCVTAB_SIZE, STC311x_REG_OCVTAB, (unsigned char *) BattData.OCVOffset);
#endif
#ifdef STC3117
	for (i=0; i < OCVTAB_SIZE; i++)
	{
		if (BattData.OcvValue[i] !=0) STC31xx_WriteWord16(STC311x_REG_OCVTAB + i*OCVTAB_BYTECOUNT, BattData.OcvValue[i]*100/55);
	}

	//if new SOCTAB value from user, overwrite the default register values.
	if (BattData.SoctabValue[1] !=0) STC31xx_Write(SOCTAB_SIZE, STC311x_REG_SOCTAB, (unsigned char *) BattData.SoctabValue);
#endif

	/* set alarm level if different from default */
	if (BattData.Alm_SOC !=0 )   
		STC31xx_WriteByte8(STC311x_REG_ALARM_SOC,BattData.Alm_SOC*2); 

	if (BattData.Alm_Vbat !=0 ) 
	{
		value= ((BattData.Alm_Vbat << 9) / VoltageFactor); /* LSB=8*2.44mV */
		STC31xx_WriteByte8(STC311x_REG_ALARM_VOLTAGE, value);
	}

	/* relaxation timer */
	if (BattData.RelaxThreshold != 0 )  
	{
		value= ((BattData.RelaxThreshold << 9) / BattData.CurrentFactor);   /* LSB=8*5.88uV/Rsense */
		value = value & 0x7f;
		STC31xx_WriteByte8(STC311x_REG_CURRENT_THRES,value); 
	}

	/* RAM restoration: set backup parameters if different from default, only if a restart is done (battery not changed) */
	if (GG_Ram.reg.CC_cnf !=0 ) STC31xx_WriteWord16(STC311x_REG_CC_CNF,GG_Ram.reg.CC_cnf); 
	if (GG_Ram.reg.VM_cnf !=0 ) STC31xx_WriteWord16(STC311x_REG_VM_CNF,GG_Ram.reg.VM_cnf); 

	STC31xx_WriteByte8(STC311x_REG_CTRL,0x03);  /*   clear PORDET, BATFAIL, free ALM pin, reset conv counter */

	if (BattData.Vmode)
		STC31xx_WriteByte8(STC311x_REG_MODE,0x19);  /*   set GG_RUN=1, voltage mode, alm enabled */
	else
		STC31xx_WriteByte8(STC311x_REG_MODE,0x18);  /*   set GG_RUN=1, mixed mode, alm enabled */

	return;
}  


/*******************************************************************************
* Function Name  : STC31xx_SaveBackupData
* Description    : Save Backup data to RAM for restoration process
* Input          : None
* Return         : -1 if error
*******************************************************************************/
int STC31xx_SaveBackupData(unsigned char * p_RamData)
{
	int status;

	/* update the RAM crc32 */
	UpdateRamCrc(p_RamData);

	status = STC311x_WriteRamData(p_RamData);
	return status;
}

/*******************************************************************************
* Function Name  : STC311x_Startup
* Description    :  initialize and start the STC311x at application startup
* Input          : None
* Return         : 0 if ok, -1 if error
*******************************************************************************/
static int STC311x_Startup(void)
{
	int res;
	int ocv, current;

	/* check STC311x status */
	res = STC311x_GetStatusWord16();
	if (res<0) return(res);

	/* read initially measured OCV */
	ocv=STC31xx_ReadWord16(STC311x_REG_OCV);

	STC311x_SetInitialParam();  /* set parameters  */

#ifdef STC3117
	/* with STC3117, it is possible here to read the current and compensate OCV: */
	current=STC31xx_ReadWord16(STC311x_REG_CURRENT);  
	current &= 0x3fff;   /* mask unused bits */
	if (current>=0x2000) current -= 0x4000;  /* convert to signed value */  
	ocv = ocv - BattData.Rint * current * 588 / BattData.Rsense / 55000 ;
#endif  

	/* rewrite ocv to start SOC with updated OCV curve */
	STC31xx_WriteWord16(STC311x_REG_OCV,ocv);

	return(0);
}


/*******************************************************************************
* Function Name  : STC311x_Restore
* Description    :  Restore STC311x state
* Input          : None
* Return         : 
*******************************************************************************/
static int STC311x_Restore(void)
{
	int res;
	int ocv;

	/* check STC311x status */
	res = STC311x_GetStatusWord16();
	if (res<0) return(res);

	/* read OCV */
	ocv=STC31xx_ReadWord16(STC311x_REG_OCV);

	STC311x_SetInitialParam();  /* set main parameters et restore RAM parameters */

#ifdef OCV_RAM_BACKUP
	/* if restore from unexpected reset, restore SOC from RAM backup (system dependent) */
	if (GG_Ram.reg.GG_Status == GG_RUNNING)
		if (GG_Ram.reg.SOC != 0)
			STC31xx_WriteWord16(STC311x_REG_SOC, GG_Ram.reg.HRSOC);  /*   restore SOC */
#else  
	/* rewrite ocv to start SOC with updated OCV curve */
	STC31xx_WriteWord16(STC311x_REG_OCV,ocv);
#endif  
	return(0);
}




/*******************************************************************************
* Function Name  : STC311x_Powerdown
* Description    :  stop the STC311x at application power down
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC311x_Powerdown(void)
{
	int res;

	/* write 0x01 into the REG_CTRL to release IO0 pin open, */
	STC31xx_WriteByte8(STC311x_REG_CTRL, 0x01);

	/* write 0 into the REG_MODE register to put the STC311x in standby mode */
	res = STC31xx_WriteByte8(STC311x_REG_MODE, 0);
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC311x_xxxx
* Description    :  misc STC311x utility functions
* Input          : None
* Return         : None
*******************************************************************************/
static void STC311x_Reset(void)
{
	STC31xx_WriteByte8(STC311x_REG_CTRL, STC311x_SOFTPOR);  /*   set soft POR */
}

static void STC311x_Reset_Adj(void)
{
	//register only available in STC3115

#ifdef STC3115
	int value;

	value=STC31xx_ReadByte8(STC311x_REG_MODE);
	STC31xx_WriteByte8(STC311x_REG_MODE,value | (STC311x_CLR_VM_ADJ+STC311x_CLR_CC_ADJ) );
#endif  
}

static void STC311x_SetSOC(int SOC)
{
	STC31xx_WriteWord16(STC311x_REG_SOC,SOC);   
}

static void STC311x_ForceVM(void) //Force Voltage Mode
{
	int value;

	value=STC31xx_ReadByte8(STC311x_REG_MODE);
	STC31xx_WriteByte8(STC311x_REG_MODE,value | STC311x_FORCE_VM);   /*   force VM mode */
}

static void STC311x_ForceCC(void) //Force Coulomb Counter Mode (Mixed mode)
{
	int value;

	value=STC31xx_ReadByte8(STC311x_REG_MODE);
	STC31xx_WriteByte8(STC311x_REG_MODE,value | STC311x_FORCE_CC);  /*   force CC mode */   
}



static int STC311x_SaveVMcnf_CCcnf(void)
{
	int reg_mode,value;

	/* mode register*/
	reg_mode = BattData.STC_Status & 0xff;

	reg_mode &= ~STC311x_GG_RUN;  /*   set GG_RUN=0 before changing algo parameters */
	STC31xx_WriteByte8(STC311x_REG_MODE, reg_mode);  

	//STC31xx_ReadByte8(STC311x_REG_ID);

	STC31xx_WriteWord16(STC311x_REG_VM_CNF,GG_Ram.reg.VM_cnf); 
	value = STC31xx_ReadWord16(STC311x_REG_SOC); 
	STC31xx_WriteWord16(STC311x_REG_SOC,value); 
	STC31xx_WriteWord16(STC311x_REG_CC_CNF,GG_Ram.reg.CC_cnf); 

	if (BattData.Vmode)
	{
		STC31xx_WriteByte8(STC311x_REG_MODE,0x19);  /*   set GG_RUN=1, voltage mode, alm enabled */
	}
	else
	{
		STC31xx_WriteByte8(STC311x_REG_MODE,0x18);  /*   set GG_RUN=1, mixed mode, alm enabled */

		if (BattData.GG_Mode == CC_MODE)
			STC31xx_WriteByte8(STC311x_REG_MODE,0x38);  /*   force CC mode */   
		else
			STC31xx_WriteByte8(STC311x_REG_MODE,0x58);  /*   force VM mode */
	}

	return(0);
}

static int STC311x_SaveVMCnf(void)
{
	int reg_mode;

	/* mode register*/
	reg_mode = BattData.STC_Status & 0xff;

	reg_mode &= ~STC311x_GG_RUN;  /*   set GG_RUN=0 before changing algo parameters */
	STC31xx_WriteByte8(STC311x_REG_MODE, reg_mode);  

	//STC31xx_ReadByte8(STC311x_REG_ID);

	STC31xx_WriteWord16(STC311x_REG_VM_CNF,GG_Ram.reg.VM_cnf); 

	if (BattData.Vmode)
	{
		STC31xx_WriteByte8(STC311x_REG_MODE,0x19);  /*   set GG_RUN=1, voltage mode, alm enabled */
	}
	else
	{
		STC31xx_WriteByte8(STC311x_REG_MODE,0x18);  /*   set GG_RUN=1, mixed mode, alm enabled */

		if (BattData.GG_Mode == CC_MODE)
			STC31xx_WriteByte8(STC311x_REG_MODE,0x38);  /*   force CC mode */   
		else
			STC31xx_WriteByte8(STC311x_REG_MODE,0x58);  /*   force VM mode */
	}

	return(0);
}





/*******************************************************************************
* Function Name  : conv
* Description    : conversion utility 
*  convert a raw 16-bit value from STC311x registers into user units (mA, mAh, mV, °C)
*  (optimized routine for efficient operation on 8-bit processors such as STM8)
* Input          : value, factor
* Return         : result = value * factor / 4096
*******************************************************************************/
static int conv(short value, unsigned short factor)
{
	int v;

	v= ( (long) value * factor ) >> 11;
	v= (v+1)/2;

	return (v);
}

/*******************************************************************************
* Function Name  : STC311x_GetUpdatedBatteryData    //STC311x_ReadBatteryData
* Description    :  utility function to read the battery data from STC311x
*                  to be called every 5s or so
* Input          : ref to BattData structure
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC311x_GetUpdatedBatteryData(STC311x_BattDataTypeDef *BattData)
{
	unsigned char data[16];
	int res;
	int value;

	res=STC311x_GetStatusWord16();
	if (res<0) return(res);  /* return if I2C error or STC3115 not responding */

	/* STC311x status */
	BattData->STC_Status = res;
	if (BattData->STC_Status & M_GGVM_MSK)
		BattData->GG_Mode = VM_MODE;   /* VM active */
	else 
		BattData->GG_Mode = CC_MODE;   /* CC active */


	/* read all registers of the device */

#ifdef STC3115
	/* read STC3115 registers 0 to 10 */
	res=STC31xx_Read(11, 0, data);
	if (res<0) return(res);  /* read failed */
#endif  
#ifdef STC3117
	/* read STC3117 registers 0 to 14 */
	res=STC31xx_Read(15, 0, data);
	if (res<0) return(res);  /* read failed */
#endif  


	/* fill the battery status data */
	{
		/* SOC */
		value=data[3]; value = (value<<8) + data[2];
		BattData->HRSOC = value;     /* result in 1/512% */

		/* conversion counter */
		value=data[5]; value = (value<<8) + data[4];
		BattData->ConvCounter = value;

		/* current */
		value=data[7]; value = (value<<8) + data[6];
		value &= 0x3fff;   /* mask unused bits */
		if (value>=0x2000) value -= 0x4000;  /* convert to signed value */
		BattData->Current = conv(value, BattData->CurrentFactor);  /* result in mA */

		/* voltage */
		value=data[9]; value = (value<<8) + data[8];
		value &= 0x0fff; /* mask unused bits */
		if (value>=0x0800) value -= 0x1000;  /* convert to signed value */
		value = conv(value,VoltageFactor);  /* result in mV */
		BattData->Voltage = value;  /* result in mV */

		/* temperature */
		value=data[10]; 
		if (value>=0x80) value -= 0x100;  /* convert to signed value */
		BattData->Temperature = value*10;  /* result in 0.1°C */

#ifdef STC3115
		/* read STC3115 registers CC & VM adj low */
		res=STC31xx_Read(2, STC311x_REG_CC_ADJ_LOW, data);
		if (res<0) return(res);  /* read failed */
		/* read STC3115 registers CC & VM adj high and OCV */
		res=STC31xx_Read(4, STC311x_REG_CC_ADJ_HIGH, &data[2]);
		if (res<0) return(res);  /* read failed */
		// data 0: CC_ADJ_L, 1: VM_ADJ_L, 2: CC_ADJ_H, 3: VM_ADJ_H, 4-5: OCV

		/* CC & VM adjustment counters */
		value=data[2]; value = (value<<8) + data[0];
		if (value>=0x8000) value -= 0x10000;  /* convert to signed value */
		BattData->CC_adj = value;//in 1/512%  
		value=data[3]; value = (value<<8) + data[1];
		if (value>=0x8000) value -= 0x10000;  /* convert to signed value */
		BattData->VM_adj = value;//in 1/512%  

		/* OCV */
		value=data[5]; value = (value<<8) + data[4];
		value &= 0x3fff; /* mask unused bits */
		if (value>=0x02000) value -= 0x4000;  /* convert to signed value */
		value = conv(value,VoltageFactor);  
		value = (value+2) / 4;  /* divide by 4 with rounding */
		BattData->OCV = value;  /* result in mV */
#endif

#ifdef STC3117

		/* Avg current */
		{
			value=data[12]; 
			value = (value<<8) + data[11];
			if (value>=0x8000) value -= 0x10000;  /* convert to signed value */

			if (BattData->Vmode==0) //mixed mode
			{
				value = conv(value, BattData->CurrentFactor);
				value = value / 4;  /* divide by 4  */
			}
			else //voltage mode
			{
				value = conv(value, BattData->CRateFactor);
			}
			BattData->AvgCurrent = value;  /* result in mA */
		}

		/* OCV */
		{
			value=data[14]; 
			value = (value<<8) + data[13];
			value &= 0x3fff; /* mask unused bits */
			if (value>=0x02000) value -= 0x4000;  /* convert to signed value */

			value = conv(value,VoltageFactor);  
			value = (value+2) / 4;  /* divide by 4 with rounding */
			BattData->OCV = value;  /* result in mV */
		}

		/* read STC3117 registers CC & VM adj */
		{
			/* CC adjustment counters */

			res=STC31xx_Read(2, STC311x_REG_CC_ADJ, data);
			if (res<0) return(res);  /* read failed */

			value=data[1]; 
			value = (value<<8) + data[0];
			if (value>=0x8000) value -= 0x10000;  /* convert to signed value */
			BattData->CC_adj = value; /* in 1/512% */


			/* VM adjustment counters */
			res=STC31xx_Read(2, STC311x_REG_VM_ADJ, data);
			if (res<0) return(res);  /* read failed */

			value=data[1]; 
			value = (value<<8) + data[0];
			if (value>=0x8000) value -= 0x10000;  /* convert to signed value */
			BattData->VM_adj = value; /* in 1/512% */
		}
#endif

		/* relax counter */
		{
			res=STC31xx_Read(1, STC311x_REG_CMONIT_COUNT, data);
			if (res<0) return(res);  /* read failed */
			BattData->RelaxTimer = data[0];
		}

	} //end fill the battery status data

	return(STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC311x_ReadRamData
* Description    : utility function to read the RAM data from STC311x
* Input          : ref to RAM data array
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC311x_ReadRamData(unsigned char *RamData)
{
	return(STC31xx_Read(RAM_SIZE, STC311x_REG_RAM, RamData));
}


/*******************************************************************************
* Function Name  : STC311x_WriteRamData
* Description    : utility function to write the RAM data into STC311x
* Input          : ref to RAM data array
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
static int STC311x_WriteRamData(unsigned char *RamData)
{
	return(STC31xx_Write(RAM_SIZE, STC311x_REG_RAM, RamData));
}

/******************************************************************************* 
* Function Name  : Interpolate
* Description    : interpolate a Y value from a X value and X, Y tables (n points)
* Input          : x
* Return         : y
*******************************************************************************/
static int interpolate(int x, int n, int const *tabx, int const *taby )
{  
	int index;
	int y;

	if (x >= tabx[0])
		y = taby[0];
	else if (x <= tabx[n-1])
		y = taby[n-1];
	else
	{
		/*  find interval */
		for (index= 1;index<n;index++)
			if (x > tabx[index]) break;
		/*  interpolate */
		y = (taby[index-1] - taby[index]) * (x - tabx[index]) * 2 / (tabx[index-1] - tabx[index]);
		y = (y+1) / 2;
		y += taby[index];
	}    
	return y;
}



/*******************************************************************************
* Function Name  : calcCRC8
* Description    : calculate the CRC8
* Input          : data: pointer to byte array, n: number of vytes
* Return         : CRC calue
*******************************************************************************/
static int calcCRC8(unsigned char *data, int n)
{
	int crc=0;   /* initial value */
	int i, j;

	for (i=0;i<n;i++)
	{
		crc ^= data[i];
		for (j=0;j<8;j++) 
		{
			crc <<= 1;
			if (crc & 0x100)  crc ^= 7;
		}
	}
	return(crc & 255);

}


/*******************************************************************************
* Function Name  : UpdateRamCrc
* Description    : calculate the RAM CRC
* Input          : pointer to Data 
* Return         : CRC value
*******************************************************************************/
static int UpdateRamCrc(unsigned char * p_RamData)
{
	int res;

	res=calcCRC8(p_RamData, RAM_SIZE-1);
	p_RamData[RAM_SIZE-1] = res;   /* last byte holds the CRC */
	return(res);
}

/*******************************************************************************
* Function Name  : Init_Backup_RAM
* Description    : Init the STC311x RAM registers with valid test word and CRC
* Input          : none
* Return         : none
*******************************************************************************/
static void Init_Backup_RAM(void)
{
	int index;

	for (index=0;index<RAM_SIZE;index++) 
		GG_Ram.db[index]=0;

	GG_Ram.reg.TestWord = RAM_TESTWORD;  /* ID partern to check RAM integrity */
	GG_Ram.reg.CC_cnf = BattData.CC_cnf;
	GG_Ram.reg.VM_cnf = BattData.VM_cnf;

	/* update the crc */
	UpdateRamCrc(GG_Ram.db);
}




/* compensate SOC with temperature, SOC in 0.1% units */
static int CompensateSOC(int value, int temp)
{
	int r, v;

	r=0;    
#ifdef TEMP_COMPENSATION_SOC
	r=interpolate(temp/10,NTEMP,TempTable,BattData.CapacityDerating);  /* for APP_TYP_CURRENT */
#endif

	v = (long) (value-r) * MAX_SOC * 2 / (MAX_SOC-r);   /* compensate */
	v = (v+1)/2;  /* rounding */
	if (v < 0) v = 0;
	if (v > MAX_SOC) v = MAX_SOC;

	return(v);
}






/*******************************************************************************
* Function Name  : MixedMode_FSM_management    //MM_FSM
* Description    : process the Gas Gauge state machine in mixed mode  (FSM = Finite-state machine)
* Input          : BattData
* Return         : 
* Affect         : Global Gas Gauge data
*******************************************************************************/
static void MixedMode_FSM_management(void)
{

	switch (BattData.BattState)
	{
	case BATT_CHARGING:
		if (BattData.AvgCurrent < CHG_MIN_CURRENT)
			BattData.BattState = BATT_ENDCHARG;        /* end of charge */
		break;

	case BATT_ENDCHARG:  /* end of charge state. check if fully charged or charge interrupted */
		if ( BattData.Current > CHG_MIN_CURRENT ) 
			BattData.BattState = BATT_CHARGING;
		else if (BattData.AvgCurrent < CHG_END_CURRENT )
			BattData.BattState = BATT_IDLE;     /* charge interrupted */
		else if ( (BattData.Current > CHG_END_CURRENT ) && ( BattData.Voltage > BATT_CHG_VOLTAGE ) )
			BattData.BattState = BATT_FULCHARG;  /* end of charge */
		break;

	case BATT_FULCHARG:  /* full charge state. wait for actual end of charge current */
		if ( (BattData.Current > CHG_MIN_CURRENT)) 
			BattData.BattState = BATT_CHARGING;  /* charge again */
		else if ( BattData.AvgCurrent < CHG_END_CURRENT ) 
		{
			if ( BattData.AvgVoltage > BATT_CHG_VOLTAGE )
			{
				/* end of charge detected */
				STC311x_SetSOC(MAX_HRSOC);
				STC311x_Reset_Adj();
				BattData.SOC=MAX_SOC;  /* 100% */
			}
			BattData.BattState = BATT_IDLE;     /* end of charge cycle */
		}
		break;

	case BATT_IDLE:  /* no charging, no discharging */
		if (BattData.Current > CHG_END_CURRENT)
		{
			BattData.BattState = BATT_CHARGING; /* charging again */
		}
		else if (BattData.Current < APP_MIN_CURRENT) 
			BattData.BattState = BATT_DISCHARG; /* discharging again */
		break;

	case BATT_DISCHARG:
		if (BattData.Current > APP_MIN_CURRENT) 
			BattData.BattState = BATT_IDLE;
		else if (BattData.AvgVoltage < BATT_MIN_VOLTAGE) 
			BattData.BattState = BATT_LOWBATT;
		break;

	case BATT_LOWBATT:  /* battery nearly empty... */
		if ( BattData.AvgVoltage > (BATT_MIN_VOLTAGE+50) )
			BattData.BattState = BATT_IDLE;   /* idle */
		else
			break;

	default:
		BattData.BattState = BATT_IDLE;   /* idle */

	} /* end switch */


}


static void CompensateVM(int temp)
{
	int r;

#ifdef TEMP_COMPENSATION_SOC
	r=interpolate(temp/10,NTEMP,TempTable,BattData.VM_TempTable);
	GG_Ram.reg.VM_cnf = (BattData.VM_cnf * r) / 100;
	STC311x_SaveVMCnf();  /* save new VM cnf values to STC311x */
#endif    
}


/*******************************************************************************
* Function Name  : VM_FSM_management
* Description    : process the Gas Gauge machine in voltage mode (FSM = Finite-state machine)
* Input          : BattData
* Return         : 
* Affect         : Global Gas Gauge data
*******************************************************************************/
static void VM_FSM_management(void)
{

#define DELTA_TEMP 30   /* 3 °C */

	/* in voltage mode, monitor temperature to compensate voltage mode gain */

	if ( ( BattData.AvgTemperature > (BattData.LastTemperature+DELTA_TEMP)) || 
		( BattData.AvgTemperature < (BattData.LastTemperature-DELTA_TEMP)) )
	{
		BattData.LastTemperature = BattData.AvgTemperature;
		CompensateVM(BattData.AvgTemperature);
	}

}




/*******************************************************************************
* Function Name  : Reset_FSM_GG
* Description    : reset the gas gauge state machine and flags
* Input          : None
* Return         : None
*******************************************************************************/
static void Reset_FSM_GG(void)
{
	BattData.BattState = BATT_IDLE;
}




/* -------------------- Algo validation functions ------------------------------------------- */


#define OGx

void SOC_correction_process(GasGauge_DataTypeDef *GG)
{
	int Var1=0;
	int Var2,Var3,Var4;
	int SOCopt;
	//unsigned char res;

#ifdef OGx

#define CURRENT_TH  (GG->Cnom/10)  
#define GAIN 10        
#define A_Var3 500    
#define VAR1MAX 64
#define VAR2MAX 128
#define VAR4MAX 128

	if (BattData.SOC>800)  Var3=600;
	else if (BattData.SOC>500) Var3=400;
	else if (BattData.SOC>250) Var3=200;
	else if (BattData.SOC>100) Var3=300;
	else Var3=400;

	Var1= 256*BattData.AvgCurrent*A_Var3/Var3/CURRENT_TH;
	Var1= 32768 * GAIN / (256+Var1*Var1/256) / 10;
	Var1 = (Var1+1)/2;
	if (Var1==0) Var1=1;
	if (Var1>=VAR1MAX) Var1=VAR1MAX-1;
	GG->Var1 = Var1;

	Var4=BattData.CC_adj-BattData.VM_adj;
	if (BattData.GG_Mode == CC_MODE)  
		SOCopt = BattData.HRSOC - BattData.CC_adj + Var1      * Var4 / 64;
	else
		SOCopt = BattData.HRSOC - BattData.VM_adj - (64-Var1) * Var4 / 64;

	Var2 = BattData.Nropt;
	if ( (BattData.AvgCurrent < -CURRENT_TH) || (BattData.AvgCurrent > CURRENT_TH) ) 
	{
		if (Var2<VAR2MAX)  Var2++;
		BattData.Ropt = BattData.Ropt + ( 1000 * (BattData.Voltage-BattData.OCV) / BattData.AvgCurrent - BattData.Ropt / Var2);
		BattData.Nropt = Var2;
	}
	if (Var2>0)
		GG->Ropt = BattData.Ropt / Var2;
	else
		GG->Ropt = 0;  // not available

	if (SOCopt <= 0 )
		SOCopt = 0;
	if (SOCopt >= MAX_HRSOC)
		SOCopt = MAX_HRSOC;
	BattData.SOC = (SOCopt*10+256)/512;
	if ( (Var4<(-VAR4MAX)) || (Var4>=VAR4MAX) )
	{
		// rewrite SOCopt into STC311x
		STC31xx_WriteWord16(STC311x_REG_SOC, SOCopt); 

		// clear acc registers
		STC311x_Reset_Adj();
	}

#endif

}

/* --------------------------------------------------------------------------------------------- */



/* -------------------- firmware interface functions ------------------------------------------- */




/*******************************************************************************
* Function Name  : GasGauge_Start
* Description    : Start the Gas Gauge system
* Input          : algo parameters in GG structure
* Return         : 0 is ok, -1 if STC310x not found or I2C error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
int GasGauge_Start(GasGauge_DataTypeDef *GG)
{
	int res, i;

	BattData.Cnom = GG->Cnom;
	BattData.Rsense = GG->Rsense;
	BattData.Rint = GG->Rint;
	BattData.Vmode = GG->Vmode;
	BattData.CC_cnf = GG->CC_cnf; 
	BattData.VM_cnf = GG->VM_cnf; 
	BattData.Alm_SOC = GG-> Alm_SOC; 
	BattData.Alm_Vbat = GG->Alm_Vbat; 
	BattData.RelaxThreshold = GG->RelaxCurrent;

	// BATD ok
	BattData.BattOnline = 1;

	if (BattData.Rsense==0) BattData.Rsense=10;  /* default value in case, to avoid divide by 0 */
	BattData.CurrentFactor=24084/BattData.Rsense;    /* LSB=5.88uV/R= ~24084/R/4096 - convert to mA  */
#ifdef STC3117
	BattData.CRateFactor=36*BattData.Cnom;        /* LSB=0.008789.Cnom= 36*Cnom/4096 - convert to mA  */
#endif

	if (BattData.CC_cnf==0) BattData.CC_cnf=395;  /* default values */
	if (BattData.VM_cnf==0) BattData.VM_cnf=321;  /* default values */

	for (i=0;i<NTEMP;i++)
		BattData.CapacityDerating[i] = GG->CapDerating[i]; 
	
	for (i=0;i<OCVTAB_SIZE;i++)
	{
#ifdef STC3115  
		BattData.OCVOffset[i] = GG->OCVOffset[i]; 
#endif
#ifdef STC3117
		BattData.OcvValue[i] = GG->OcvValue[i]; 
		BattData.SoctabValue[i] = GG->SoctabValue[i]; 
#endif
	}	
	
	for (i=0;i<NTEMP;i++)
		BattData.VM_TempTable[i] = DefVMTempTable[i];    

	BattData.Ropt = 0;
	BattData.Nropt = 0;

	/* check STC311x status */
	res = STC311x_GetStatusWord16();
	if (res<0) return(res);


	/* check RAM valid */
	STC311x_ReadRamData(GG_Ram.db);

	if ( (GG_Ram.reg.TestWord != RAM_TESTWORD) || (calcCRC8(GG_Ram.db,RAM_SIZE) != 0) )
	{
		/* RAM invalid, so perform a totally new initialization */
		Init_Backup_RAM();
		res=STC311x_Startup();  /* return -1 if I2C error or STC3115 not present */
	}
	else 
	{
		/* check STC311x status */
		if ((STC311x_GetStatusWord16() & M_RST_ERR_MSK) != 0 ) //Alert condition has occured. Need to reset the Alerts.
		{
			res=STC311x_Startup();  /* return -1 if I2C error or STC3115 not present */
		}
		else //RAM backup content is valid, so use the restoration process to improve accuracy
		{
			res=STC311x_Restore(); /* recover from last SOC */
		}
	}


	GG_Ram.reg.GG_Status = GG_INIT;
	STC31xx_SaveBackupData(GG_Ram.db);

	Reset_FSM_GG();

	return(res);    /* return -1 if I2C error or STC3115 not present */
}





/*******************************************************************************
Restart sequence:
Usage: 
call GasGaugeReset()
powerdown everything
wait 500ms
call GasGaugeStart(GG)
continue 
*******************************************************************************/


/*******************************************************************************
* Function Name  : GasGauge_Reset
* Description    : Reset the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if I2C error
*******************************************************************************/
void GasGauge_Reset(void)  
{
	GG_Ram.reg.TestWord=0;  /* reset RAM */
	GG_Ram.reg.GG_Status = 0;
	STC311x_WriteRamData(GG_Ram.db);

	STC311x_Reset();
}



/*******************************************************************************
* Function Name  : GasGauge_Stop
* Description    : Stop the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if I2C error
*******************************************************************************/
int GasGauge_Stop(void)
{
	int res;

	STC311x_ReadRamData(GG_Ram.db);
	GG_Ram.reg.GG_Status= GG_POWERDN;

	STC31xx_SaveBackupData(GG_Ram.db);

	res=STC311x_Powerdown();
	if (res!=0) return (-1);  /* error */

	return(0);  
}



/*******************************************************************************
* Function Name  : GasGauge_Task
* Description    : Periodic Gas Gauge task, to be called e.g. every 5 sec.
* Input          : pointer to gas gauge data structure
* Return         : 1 if data available, 0 if no data, -1 if error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
int GasGauge_Task(GasGauge_DataTypeDef *GG)
{
	int res, value;

	BattData.Rsense = GG->Rsense;
	BattData.Vmode = GG->Vmode;
	BattData.Alm_SOC = GG-> Alm_SOC; 
	BattData.Alm_Vbat = GG->Alm_Vbat; 
	BattData.RelaxThreshold = GG->RelaxCurrent;

	res=STC311x_GetUpdatedBatteryData(&BattData);  /* read battery data into global variables */
	if (res!=0) return(-1); /* abort in case of I2C failure */

	/* check if RAM data is ok (battery has not been changed) */
	STC311x_ReadRamData(GG_Ram.db);
	if ( (GG_Ram.reg.TestWord!= RAM_TESTWORD) || (calcCRC8(GG_Ram.db,RAM_SIZE)!=0) )
	{
		/* if RAM not ok, reset it and set init state */
		Init_Backup_RAM(); 
		GG_Ram.reg.GG_Status = GG_INIT;
	}    

	//Check battery presence
	if ((BattData.STC_Status & M_BATFAIL_MSK) != 0)
	{
		BattData.BattOnline = 0;  //battery not connected
	}

#ifdef BATD_UC8
	/* check STC3115 status */
	if ((BattData.STC_Status & M_BATFAIL_MSK) != 0)
	{
		/* BATD or UVLO detected */
		if(BattData.ConvCounter > 0)
		{
			GG->Voltage=BattData.Voltage;
			GG->SOC=(BattData.HRSOC*10+256)/512;
		}

		/* BATD or UVLO detected */
		GasGauge_Reset();

		return (-1);
	}
#endif

	if ((BattData.STC_Status & M_RUN_MSK) == 0)
	{
		/* if not running, restore STC3115 */
		STC311x_Restore();  
		GG_Ram.reg.GG_Status = GG_INIT;
	}

	BattData.SOC = (BattData.HRSOC*10+256)/512;  /* in 0.1% unit  */

	//Force an external temperature
	if(GG->ForceExternalTemperature == 1)
		BattData.Temperature = GG->ExternalTemperature;

	/* check INIT state */
	if (GG_Ram.reg.GG_Status == GG_INIT)
	{
		/* INIT state, wait for current & temperature value available: */
		if (BattData.ConvCounter>VCOUNT) 
		{
			/* update VM_cnf */
			CompensateVM(BattData.Temperature);
			BattData.LastTemperature=BattData.Temperature;

			/* Init averaging */
			BattData.AvgVoltage = BattData.Voltage;
			BattData.AvgCurrent = BattData.Current;
			BattData.AvgTemperature = BattData.Temperature;
			BattData.AvgSOC = CompensateSOC(BattData.SOC,BattData.Temperature);  /* in 0.1% unit  */
			BattData.AccVoltage = BattData.AvgVoltage*AVGFILTER;
			BattData.AccCurrent = BattData.AvgCurrent*AVGFILTER;
			BattData.AccTemperature = BattData.AvgTemperature*AVGFILTER;
			BattData.AccSOC = BattData.AvgSOC*AVGFILTER;

			GG_Ram.reg.GG_Status = GG_RUNNING;
		}
	}


	if (GG_Ram.reg.GG_Status != GG_RUNNING)
	{
		GG->SOC = CompensateSOC(BattData.SOC,250);
		GG->Voltage=BattData.Voltage;
		GG->OCV = BattData.OCV;
		GG->Current=0;
		GG->RemTime = -1;   /* means no estimated time available */
		GG->Temperature=250;
	}
	else //running OK
	{
		//Check battery presence
		if ((BattData.STC_Status & M_BATFAIL_MSK) == 0)
		{
			BattData.BattOnline = 1;  //battery connected
		}

		SOC_correction_process(GG);
		/* SOC derating with temperature */
		BattData.SOC = CompensateSOC(BattData.SOC,BattData.Temperature);

		//early empty compensation
		value=BattData.AvgVoltage;
		if (BattData.Voltage < value) value = BattData.Voltage;
		if (value<(APP_MIN_VOLTAGE+200) && value>(APP_MIN_VOLTAGE-500))
		{
			if (value<APP_MIN_VOLTAGE) 
				BattData.SOC=0;
			else
				BattData.SOC = BattData.SOC * (value - APP_MIN_VOLTAGE) / 200;
		}

		BattData.AccVoltage += (BattData.Voltage - BattData.AvgVoltage);
		BattData.AccCurrent += (BattData.Current - BattData.AvgCurrent);
		BattData.AccTemperature += (BattData.Temperature - BattData.AvgTemperature);
		BattData.AccSOC +=  (BattData.SOC - BattData.AvgSOC);

		BattData.AvgVoltage = (BattData.AccVoltage+AVGFILTER/2)/AVGFILTER;
#ifdef STC3115
		BattData.AvgCurrent = (BattData.AccCurrent+AVGFILTER/2)/AVGFILTER;
#endif	
		BattData.AvgTemperature = (BattData.AccTemperature+AVGFILTER/2)/AVGFILTER;
		BattData.AvgSOC = (BattData.AccSOC+AVGFILTER/2)/AVGFILTER;



		/* ---------- process the Gas Gauge algorithm -------- */

		if (BattData.Vmode) 
			VM_FSM_management();  /* in voltage mode */
		else
			MixedMode_FSM_management();  /* MM_FSM :in mixed mode */

		if (BattData.Vmode==0) 
		{
			// Lately fully compensation
			if(BattData.AvgCurrent > 0 && BattData.SOC >= 990 && BattData.SOC < 995 && BattData.AvgCurrent > 100)
			{
				BattData.SOC = 990;
				STC311x_SetSOC(99*512);
			}
			// Lately empty compensation
			if(BattData.AvgCurrent < 0 && BattData.SOC >= 15 && BattData.SOC < 20 && BattData.Voltage > (APP_MIN_VOLTAGE+50))
			{
				BattData.SOC = 20;
				STC311x_SetSOC(2*512);
			}
		}


		/* -------- APPLICATION RESULTS ------------ */

		/* fill gas gauge data with battery data */
		GG->Voltage=BattData.Voltage;
		GG->Current=BattData.Current;
		GG->Temperature=BattData.Temperature;
		GG->SOC = BattData.SOC;
		GG->OCV = BattData.OCV;

		GG->AvgVoltage = BattData.AvgVoltage;
		GG->AvgCurrent = BattData.AvgCurrent;
		GG->AvgTemperature = BattData.AvgTemperature;
		GG->AvgSOC = BattData.AvgSOC;

		if (BattData.Vmode) 
		{
			/* no current value in voltage mode */
#ifdef STC3115
			GG->Current = 0;
			GG->AvgCurrent = 0;
#endif
		}

		GG->ChargeValue = (long) BattData.Cnom * BattData.AvgSOC / MAX_SOC;
		if (GG->AvgCurrent<APP_MIN_CURRENT)
		{
			GG->State=BATT_DISCHARG;
			value = GG->ChargeValue * 60 / (-GG->AvgCurrent);  /* in minutes */
			if (value<0) value=0;
			GG->RemTime = value; 
		}
		else 
		{
			GG->RemTime = -1;   /* -1 means no estimated time available */
			if (GG->AvgCurrent>CHG_END_CURRENT)
				GG->State=BATT_CHARGING;
			else
				GG->State=BATT_IDLE;
		}
	}

	/* save periodically the last valid SOC in the embedded RAM */
	{
		GG_Ram.reg.HRSOC = BattData.HRSOC;
		GG_Ram.reg.SOC = (GG->SOC+5)/10;    /* trace SOC in % */
		STC31xx_SaveBackupData(GG_Ram.db);
	}

	if (GG_Ram.reg.GG_Status==GG_RUNNING)
		return(1); //OK
	else
		return(0);  /* only SOC, OCV and voltage are valid */
}




/*******************************************************************************
* Function Name  : STC31xx_SetPowerSavingMode
* Description    :  Set the power saving mode (i.e. in Voltage Mode only)
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_SetPowerSavingMode(void)
{
	int res;

	/* Read the mode register*/
	res = STC31xx_ReadByte8(STC311x_REG_MODE);

	/* Set the VMODE bit to 1 */
	res = STC31xx_WriteByte8(STC311x_REG_MODE, (res | STC311x_VMODE));
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC31xx_StopPowerSavingMode
* Description    :  Stop the power saving mode (i.e. go in Mixed Mode)
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_StopPowerSavingMode(void)
{
	int res;

	/* Read the mode register*/
	res = STC31xx_ReadByte8(STC311x_REG_MODE);

	/* Set the VMODE bit to 0 */
	res = STC31xx_WriteByte8(STC311x_REG_MODE, (res & ~STC311x_VMODE));
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC31xx_AlarmSet
* Description    :  Set the alarm function and set the alarm threshold
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_AlarmSet(void)
{
	int res;

	/* Read the mode register*/
	res = STC31xx_ReadByte8(STC311x_REG_MODE);

	/* Set the ALM_ENA bit to 1 */
	res = STC31xx_WriteByte8(STC311x_REG_MODE, (res | STC311x_ALM_ENA));
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC31xx_AlarmStop
* Description    :  Stop the alarm function
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_AlarmStop(void)
{
	int res;

	/* Read the mode register*/
	res = STC31xx_ReadByte8(STC311x_REG_MODE);

	/* Set the ALM_ENA bit to 0 */
	res = STC31xx_WriteByte8(STC311x_REG_MODE, (res & ~STC311x_ALM_ENA));
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}


/*******************************************************************************
* Function Name  : STC31xx_AlarmGet
* Description    : Return the ALM status
* Input          : None
* Return         : ALM status 00 : no alarm 
*                             01 : SOC alarm
*                             10 : Voltage alarm
*                             11 : SOC and voltage alarm
*******************************************************************************/
int STC31xx_AlarmGet(void)
{
	int res;

	/* Read the mode register*/
	res = STC31xx_ReadByte8(STC311x_REG_CTRL);
	res = res >> 5;

	return (res);
}


/*******************************************************************************
* Function Name  : STC31xx_AlarmClear
* Description    :  Clear the alarm signal
* Input          : None
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_AlarmClear(void)
{
	int res;

	/* clear ALM bits*/
	res = STC31xx_WriteByte8(STC311x_REG_CTRL, 0x01);
	if (res!= STC31XX_OK) return (res);

	return (res);
}


/*******************************************************************************
* Function Name  : STC31xx_AlarmSetVoltageThreshold
* Description    : Set the alarm threshold
* Input          : int voltage threshold
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_AlarmSetVoltageThreshold(int VoltThresh)
{
	int res;
	int value;

	BattData.Alm_Vbat =VoltThresh;

	value= ((BattData.Alm_Vbat << 9) / VoltageFactor); /* LSB=8*2.44mV */
	res = STC31xx_WriteByte8(STC311x_REG_ALARM_VOLTAGE, value);
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}




/*******************************************************************************
* Function Name  : STC31xx_AlarmSetSOCThreshold
* Description    : Set the alarm threshold
* Input          : int voltage threshold
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_AlarmSetSOCThreshold(int SOCThresh)
{
	int res;

	BattData.Alm_SOC = SOCThresh;
	res = STC31xx_WriteByte8(STC311x_REG_ALARM_SOC, BattData.Alm_SOC*2);
	if (res!= STC31XX_OK) return (res);

	return (STC31XX_OK);
}




/*******************************************************************************
* Function Name  : STC31xx_RelaxTmrSet
* Description    :  Set the current threshold register to the passed value in mA
* Input          : int current threshold
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_RelaxTmrSet(int CurrentThreshold)
{
	int res, value;

	BattData.RelaxThreshold = CurrentThreshold;
	if (BattData.CurrentFactor!=0) 
	{
		value= ((BattData.RelaxThreshold << 9) / BattData.CurrentFactor);   /* LSB=8*5.88uV/Rsense */
		value = value & 0x7f;
		res=STC31xx_WriteByte8(STC311x_REG_CURRENT_THRES,value);     
		if (res!= STC31XX_OK) return (res);
	}

	return (STC31XX_OK);
}

/*******************************************************************************
* Function Name  : STC31xx_ForceMixedMode
* Description    :  Force the CC mode for CC eval
* Input          : 
* Return         : error status (STC31XX_OK, !STC31XX_OK)
*******************************************************************************/
int STC31xx_ForceMixedMode(void)
{
	STC311x_ForceCC();

	return (STC31XX_OK);
}







/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
