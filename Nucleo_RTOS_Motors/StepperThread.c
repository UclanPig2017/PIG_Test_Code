/**
  ******************************************************************************
  * File Name          : StepperThread.c
  * Description        : Stepper Control Code
	* User Code by			 : Luke Jackson, Sulaymaan Shaikh
	* Version						 : 0.2
	*
	* Changelog:
	*						0.3:
	*							- Reviewed and corrected stepper pins (again)
	*						0.2:
	*							- Corrected Pins
	*						0.1:
	*							- Added Thread Init Code
	*							- Integrated Luke+Sully Stepper Motor Drive code
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
int state = 1;
int stepDir = 0;
int limit1 = 1;
int limit2 = 1;
int limit4 = 1;
int limit13 = 1;
int limit14 = 1;
int limit15 = 1;

/* FUNCTION PROTOTYPES */
extern void driverShutdown(void);
extern void strainShutdown(void);

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
			//Retract Legs until minimum limit switches pressed
		 while (!limit1 & !limit2 & !limit4)
		 {
			switch (state) 
			{
				case 1:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); 
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
				
					break;
				}
			
				case 2:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
				
					break;
				}
			
				case 3:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
					
					break;
				}
			
				case 4:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
				
					break;
				}
			
				default:
					break;
			}
			
			state--;
			
			if (state < 1)
			{
				state = 4;
			}
			
		 }
			//Reset pigShutdown
			pigShutdown = 0;
		 
			//Shutdown Driving Motors
			driverShutdown();
			//Reset Strain Gauge Thread
			strainShutdown();
			//Yield Thread
			osThreadYield();
		}
	
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
	
		//If not idling, shift through stepper state
		if ((!limit1 | !limit2 | !limit4) && (stepDir == 0))
		{;}
		else if ((!limit13 | !limit14 | !limit15) && (stepDir == 1))
		{;}	
		else if (stepDir == 2)
		{;}
		else
		{
			//State Machine for driving step magnets
			switch (state) 
			{
				case 1:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); 
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
				
					break;
				}
			
				case 2:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
				
					break;
				}
			
				case 3:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
					
					break;
				}
			
				case 4:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
				
					break;
				}
			
				default:
					break;
			}
		
			//Increment or Decrement depending on stepdirection
			switch (stepDir)
			{
				case 0:
					state++;
					break;
				case 1:
					state--;
					break;
				default:
					break;
			}
		
			//Reset State if outside state bounds
			if (state > 4)
			{
				state = 1;
			}
			else if (state < 1)
			{
				state = 4;
			}
		}
		
		//Delay drive for 1ms
		HAL_Delay(1);
		osThreadYield();	//Yield Thread
	}
}
