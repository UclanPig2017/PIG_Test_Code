/**
  ******************************************************************************
  * File Name          : StrainGaugeThread.c
  * Description        : Strain Gauge - Determine direction of stepper
	* User Code by			 : Ian Taylor, Luke Jackson, Sulaymaan Shaikh
	* Version						 : 0.1
	*
	* Changelog:
	*						0.1:
	*							- Added Strain Gauge Code
  ******************************************************************************
	*/
	
/* INCLUDE FILES */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"                   // CMSIS RTOS header file

/* EXTERNAL VARIABLES*/
extern int pigStatus;										// STATUS DETERMINES IF SYSTEM NEEDS TO RUN (1) OR SHUTDOWN (0)
extern int pigShutdown;									// SHUTDOWN DETERMINES IF NEED TO RESET VARIABLES
extern uint16_t adcBuf[7];
extern int stepDir;
extern int convComplete;								
float aveStrain, prevStrain;
/* LOCAL VARIABLES */
uint16_t strainGauge[3];
uint16_t strainGaugePrev[3];
int firstRun = 1;

/* FUNCTION PROTOTYPES */
void strainShutdown(void);

/* CODE */

//Thread Initialisation
void StrainGauge(void const *argument);                     	// thread function
osThreadId tid_StrainGauge;                                  // thread id
osThreadDef (StrainGauge, osPriorityNormal, 1, 0);           // thread object

/**
  * @brief  This function initialises the Strain Gauge read thread
  * @param  None
  * @retval Thread Initialisation Status
  */
int initStrainGauge (void) 
{
  tid_StrainGauge = osThreadCreate (osThread(StrainGauge), NULL);
  if (!tid_StrainGauge) return(-1);
  return(0);
}

//Thread Main Code
void StrainGauge(void const *argument)
{
	while(1)
	{
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
			
		while(firstRun)
		{
			osThreadYield();
		}
		
		while(!convComplete)
		{
			osThreadYield();
		}
		
		//Begin Processing Strain Values
		// Average strain 
		
		prevStrain = ((strainGaugePrev[0] + strainGaugePrev[1] + strainGaugePrev[2]) /3);
		
		aveStrain = ((strainGauge[0] + strainGauge[1] + strainGauge[2])/3);
		
		if((aveStrain < (prevStrain*1.1))&&(aveStrain > (prevStrain*0.90)))
		{
			stepDir = 2;
		}
		else if (aveStrain > (prevStrain*1.1))
		{
			stepDir = 1;
		}
		else if (aveStrain < (prevStrain*0.9))
		{
			stepDir = 0;
		}
		
		//Set new values as previous values
		strainGaugePrev[0] = strainGauge[0];
		strainGaugePrev[1] = strainGauge[1];
		strainGaugePrev[2] = strainGauge[2];
		
		//Yield Thread
		osThreadYield();
	}
}

/**
  * @brief  This function resets the strain gauge values
  * @param  None
  */
void strainShutdown(void)
{
	strainGaugePrev[0] = 0; 
	strainGaugePrev[1] = 0;
	strainGaugePrev[2] = 0;
	
	strainGauge[0] = 0; 
	strainGauge[1] = 0;
	strainGauge[2] = 0;
	firstRun = 1;
}
