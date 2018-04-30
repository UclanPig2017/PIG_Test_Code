/**
  ******************************************************************************
  * File Name          : EncoderThread.c
  * Description        : Encoder Reading Code
	* User Code by			 : Tom Harrison, Ian Taylor, Sulaymaan Shaikh
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
void EncoderRead (void const *argument);                     // thread function
osThreadId tid_EncoderRead;                                  // thread id
osThreadDef (EncoderRead, osPriorityNormal, 1, 0);           // thread object

/**
  * @brief  This function initialises the Encoder Read thread
  * @param  None
  * @retval Thread Initialisation Status
  */
int initEncoders (void) 
{
  tid_EncoderRead = osThreadCreate (osThread(EncoderRead), NULL);
  if (!tid_EncoderRead) return(-1);
  return(0);
}

//Thread Main Code
void EncoderRead (void const *argument)
{
	while(1)
	{	
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
	
		//If running, read the Encoders
		
		osThreadYield();	//Yield Thread
	}
}
