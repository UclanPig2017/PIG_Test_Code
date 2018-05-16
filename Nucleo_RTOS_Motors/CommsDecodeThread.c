/**
  ******************************************************************************
  * File Name          : CommmsDecodeThread.c
  * Description        : Decoding Commands received via UART
	* User Code by			 : Sulaymaan Shaikh
	* Version						 : 0.3
	*
	* Changelog:
	*						0.3:
	*							- Set pigDir to 0 for shutting down drive motors
	*						0.2:
	*							- Added Decode and variable setting for drive motors
	*						0.1:
	*							- Added Thread Init Code
	*							- Preliminary Decode
  ******************************************************************************
*/
	
/* Include Files*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
	
/* Variables */
extern uint8_t rxBuf[1];
extern int pigDir;
int pigShutdown = 0;
int pigStatus = 0;
int ComRec = 0;
	
/* Function Prototypes*/

/* CODE */

//Thread Initialisation
void CommsDecode (void const *argument);                     // thread function
osThreadId tid_CommsDecode;                                  // thread id
osThreadDef (CommsDecode, osPriorityNormal, 1, 0);           // thread object
	
int initCommsDecode (void) 
{
	tid_CommsDecode = osThreadCreate (osThread(CommsDecode), NULL);
	if (!tid_CommsDecode) return(-1);
	return(0);
}

//Thread Main Code
void CommsDecode (void const *argument)
{
	while(1)
	{	
		//Check if Command Received
		if (ComRec)
		{
			switch (rxBuf[0]){
				case '0':
					//Activate Threads
					pigStatus = 1;
					break;
				case '1':
					//Go Forward
					pigDir = 1;
					break;
				case '2':
					//Go Backward
					pigDir = 2;
					break;
				case '3':
					//Stop
					pigDir = 0;
					break;
				case '4':
					//Send Data
					break;
				case '5':
					//Retract Stepper, Idle Threads
					pigDir = 0
					pigShutdown = 1;
					pigStatus = 0;
					break;
				default:
					break;
			}
			ComRec = 0;
		}				
		
		osThreadYield();	//Yield Thread
	}
}
