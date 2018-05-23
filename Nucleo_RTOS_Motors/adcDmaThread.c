/**
  ******************************************************************************
  * File Name          : adcDmaThread.c
  * Description        : ADC Read Code
	* User Code by			 : Sulaymaan Shaikh
	* Version						 : 0.2
	*
	* Changelog:
	*						0.1:
	*							- Added ADC DMA Reading Code
  ******************************************************************************
	*/
	
/* INCLUDE FILES */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"                   // CMSIS RTOS header file

/* EXTERNAL VARIABLES*/
extern int pigStatus;										// STATUS DETERMINES IF SYSTEM NEEDS TO RUN (1) OR SHUTDOWN (0)
extern ADC_HandleTypeDef hadc1;
extern uint16_t strainGauge[3];
extern uint16_t strainGaugePrev[3];
extern int firstRun;

/* LOCAL VARIABLES */
uint16_t adcBuf[7];
int convComplete = 0;

/* FUNCTION PROTOTYPES */

/* CODE */
//Thread Initialisation
void adcRead (void const *argument);                     // thread function
osThreadId tid_adcRead;                                  // thread id
osThreadDef (adcRead, osPriorityNormal, 1, 0);           // thread object

/**
  * @brief  This function initialises the ADC Read thread
  * @param  None
  * @retval Thread Initialisation Status
  */
int initAdcRead (void) 
{
  tid_adcRead = osThreadCreate (osThread(adcRead), NULL);
  if (!tid_adcRead) return(-1);
  return(0);
}

//Thread Main Code
void adcRead (void const *argument)
{
	while(1)
	{
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
		
		convComplete = 0;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuf, 7);
		HAL_Delay(100);		
		
		//Get initial strain values
		if (firstRun)
		{
			strainGaugePrev[0] = adcBuf[0]; 
			strainGaugePrev[1] = adcBuf[1];
			strainGaugePrev[2] = adcBuf[2];
			
			firstRun = 0;
		}
		else
		{
			//Get Current Strain Values
			strainGauge[0] = adcBuf[0]; 
			strainGauge[1] = adcBuf[1];
			strainGauge[2] = adcBuf[2];	
			convComplete = 1;
		}
		
		osThreadYield();
	}
}
