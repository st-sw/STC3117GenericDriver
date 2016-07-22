/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : stc311x_Gasgauge.h
* Author             : AMS application
* Version            : V2.07
* Date               : 2015/09/01
* Description        : Header for stc311x_gasgauge.c module (STC3117 gas gauge)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

#ifdef __cplusplus      //c++
extern "C"              //c++
{                       //c++
#endif                  //c++




/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __Gasgauge_H
#define __Gasgauge_H

//C++ modification #include "i2c.h"

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

typedef struct  {
  int Voltage;        /* battery voltage in mV */
  int Current;        /* battery current in mA */
  int Temperature;    /* battery temperature in 0.1°C */
  int SOC;            /* battery relative SOC (%) in 0.1% */
  int OCV;
  int AvgSOC;
  int AvgCurrent;
  int AvgVoltage;
  int AvgTemperature;
  int ChargeValue;    /* remaining capacity in mAh */
  int RemTime;        /* battery remaining operating time during discharge (min) */
  int State;          /* charge (>0)/discharge(<0) state */
  int CalStat;        /* Internal status */
  /* -- parameters -- */
  int Vmode;       /* 1=Voltage mode, 0=mixed mode */
  int Alm_SOC;     /* SOC alm level */
  int Alm_Vbat;    /* Vbat alm level */
  int CC_cnf;      /* nominal CC_cnf */
  int VM_cnf;      /* nominal VM cnf */
  int Cnom;        /* nominal capacity in mAh */
  int Rsense;      /* sense resistor */
  int Rint;         /* battery internal resistance */
  int RelaxCurrent; /* current for relaxation (< C/20) */
  int Adaptive;     /* adaptive mode */
  int CapDerating[7];   /* capacity derating in 0.1%, for temp = 60, 40, 25, 10,   0, -10, -20 °C */
#ifdef STC3115
  int OCVOffset[16];   /* OCV curve adjustment */
#else
  int OcvValue[16];       /* OCV curve values */
  int SoctabValue[16];    /* SOC curve values */
#endif
  int ExternalTemperature;
  int ForceExternalTemperature;
  int Ropt;  
  int Var1;
} GasGauge_DataTypeDef;



/* Exported functions prototypes--------------------------------------------- */

int GasGauge_Task(GasGauge_DataTypeDef *GG);
int GasGauge_Start(GasGauge_DataTypeDef *GG);
void GasGauge_Reset(void)  ;
int GasGauge_Stop(void);

int STC31xx_SetPowerSavingMode(void);
int STC31xx_StopPowerSavingMode(void);

int STC31xx_AlarmSet(void);
int STC31xx_AlarmStop(void);
int STC31xx_AlarmGet(void);
int STC31xx_AlarmClear(void);
int STC31xx_AlarmSetVoltageThreshold(int VoltThresh);
int STC31xx_AlarmSetSOCThreshold(int SOCThresh);

int STC31xx_RelaxTmrSet(int CurrentThreshold);

int STC31xx_ForceCC(void);

int STC31xx_CheckI2cDeviceId(void);
int STC31xx_GetRunningCounter(void);

#endif /* __Gasgauge_H */

#ifdef __cplusplus  //c++
}                   //c++
#endif              //c++
 
 


/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
