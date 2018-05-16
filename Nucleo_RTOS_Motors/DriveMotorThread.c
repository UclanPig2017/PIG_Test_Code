/**
  ******************************************************************************
  * File Name          : DriveMotorThread.c
  * Description        : Drive Motor Control Code
	* User Code by			 : Myles Flanagan, Callum Small, Sulaymaan Shaikh
	* Version						 : 0.1
	*
	* Changelog:
	*						0.1:
	*							- Added Thread Init Code
	*							- Integrated Callum+Myles Code
  ******************************************************************************
	*/
	
/* INCLUDE FILES */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"                   // CMSIS RTOS header file

/* EXTERNAL VARIABLES*/
extern int pigStatus;										// STATUS DETERMINES IF SYSTEM NEEDS TO RUN (1) OR SHUTDOWN (0)
extern int pigShutdown;									// SHUTDOWN DETERMINES IF ARMS NEED TO RETRACT, (1) TO RETRACT, (0)
extern TIM_HandleTypeDef htim2;


/* LOCAL VARIABLES */
int pigDir;

/* FUNCTION PROTOTYPES */
static void setPWM1(uint16_t); 
static void setPWM2(uint16_t);  
static void setPWM3(uint16_t);
	
/* CODE */

//Thread Initialisation
void DriveControl(void const *argument);                     	// thread function
osThreadId tid_DriveControl;                                  // thread id
osThreadDef (DriveControl, osPriorityNormal, 1, 0);           // thread object

/**
  * @brief  This function initialises the Stepper Motor Control thread
  * @param  None
  * @retval Thread Initialisation Status
  */
int initDriveControl (void) 
{
  tid_DriveControl= osThreadCreate (osThread(DriveControl), NULL);
  if (!tid_DriveControl) return(-1);
  return(0);
}

//Thread Main Code
void DriveControl(void const *argument)
{
	while(1)
	{	
		//If idling, yield thread
		while(!pigStatus)
		{
			osThreadYield();
		}
		
		//Driving Code
		HAL_Delay(1000); 
		
		if(pigDir == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			
			//Generate PWM 
			setPWM1(200);
			setPWM2(200);
			setPWM3(200);
		}
		
		else if(pigDir == 2)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			
			//Generate PWM Signal
			setPWM1(200);
			setPWM2(200);
			setPWM3(200);
		}
		
		else if(!pigDir)
		{
			setPWM1(0);
			setPWM2(0);
			setPWM3(0);
		}
		
		osThreadYield();	//Yield Thread
	}
}

void setPWM1(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // start pwm generation
} 

void setPWM2(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // start pwm generation
} 

void setPWM3(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // start pwm generation
} 
