/**
  ******************************************************************************
  * File Name          : StepperThread.c
  * Description        : Stepper Control Code
	* User Code by			 : <add names here>, Sulaymaan Shaikh
	* Version						 : 0.1
	*
	* Changelog:
	*						0.1:
	*							- Added Thread Init Code
  ******************************************************************************
	*/
	
/* INCLUDE FILES */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"                                           // CMSIS RTOS header file

/* EXTERNAL VARIABLES*/
extern int pigStatus;										// STATUS DETERMINES IF SYSTEM NEEDS TO RUN (1) OR SHUTDOWN (0)
extern int pigShutdown;									// SHUTDOWN DETERMINES IF ARMS NEED TO RETRACT, (1) TO RETRACT, (0)

/* LOCAL VARIABLES */

/* FUNCTION PROTOTYPES */
	
/* CODE */

//Thread Initialisation
void StepperControl (void const *argument);                     // thread function
osThreadId tid_StepperControl;                                  // thread id
osThreadDef (StepperControl, osPriorityNormal, 1, 0);           // thread object

/**
  * @brief  This function initialises the Stepper Motor Control thread
  * @param  None
  * @retval Thread Initialisation Status
  */
int initSteppers (void) 
{
  tid_StepperControl = osThreadCreate (osThread(StepperControl), NULL);
  if (!tid_StepperControl) return(-1);
  return(0);
}

//Thread Main Code
void StepperControl (void const *argument)
{
	while(1)
	{
		//If shutdown command received, retract, then yield thread
		if (pigShutdown)
		{
			//Retract Legs
		
			//Reset pigShutdown
			pigShutdown = 0;
			
			//Yield Thread
			osThreadYield();
		}
	
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
	
		//If running, control the steppers
		osThreadYield();	//Yield Thread
	}
}
