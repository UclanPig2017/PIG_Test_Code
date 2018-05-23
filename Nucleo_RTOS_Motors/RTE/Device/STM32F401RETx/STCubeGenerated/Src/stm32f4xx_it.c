/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
/**
  ******************************************************************************
  * File Name          : stm32f4xx_it.c
  * Description        : Interrupt Service Routines
	* User Code by			 : Sulaymaan Shaikh, Luke Jackson
	* Version						 : 0.2
	*
	* Changelog:
	*						0.2:
	*								- Corrected Stepper Limit code for right pins
	*								- Generated ADC DMA Code
	*						0.1:
	*								- Added code to handle UART Interrupt + Set flag
	*								- Added code to handle limit switch press/depress
  ******************************************************************************
	*/

//UART Control
extern int ComRec;
extern uint8_t rxBuf[1];

//Stepper Control
int limsw1, limsw2, limsw4, limsw13, limsw14, limsw15;
extern int limit1, limit2, limit4, limit13, limit14, limit15;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
//void SVC_Handler(void)
//{
//  /* USER CODE BEGIN SVCall_IRQn 0 */

//  /* USER CODE END SVCall_IRQn 0 */
//  /* USER CODE BEGIN SVCall_IRQn 1 */

//  /* USER CODE END SVCall_IRQn 1 */
//}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */

//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */

//  /* USER CODE END PendSV_IRQn 1 */
//}

/**
* @brief This function handles System tick timer.
*/
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */

//  /* USER CODE END SysTick_IRQn 0 */
//  HAL_IncTick();
//  HAL_SYSTICK_IRQHandler();
//  /* USER CODE BEGIN SysTick_IRQn 1 */

//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	int pinStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	if (!pinStatus)
	{
		limsw1 = 0;
		limit1 = 0;
	}
	else if (pinStatus)
	{
		limsw1 = 1;
		limit1 = 1;
	}	
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

	int pinStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	if (!pinStatus)
	{
		limsw2 = 0;
		limit2 = 0;
	}
	else if (pinStatus)
	{
		limsw2 = 1;
		limit2 = 1;
	}	
	
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

	int pinStatus = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
	if (!pinStatus)
	{
		limsw4 = 0;
		limit4 = 0;
	}
	else if (pinStatus)
	{
		limsw4 = 1;
		limit4 = 1;
	}		
	
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	//Determine if waiting on data reception 
	int waitReceive = 0;
	if ((USART2->SR & USART_SR_RXNE) != RESET)
	{
		waitReceive = 1;
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	/* FOR TRANSMISSION */
	
	/* FOR RECEPTION */
	if(waitReceive)
	{
		ComRec = 1;
		//If all characters received, restart reception
		HAL_UART_Receive_IT(&huart2, rxBuf, 1);
	}
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
	{
		int pinStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		if (!pinStatus)
		{
			limsw13 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			limit13 = 0;
		}
		else if (pinStatus)
		{
			limsw13 = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			limit13 = 1;
		}		
	}
	
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
	{
		int pinStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		if (!pinStatus)
		{
			limsw14 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			limit14 = 0;
		}
		else if (pinStatus)
		{
			limsw14 = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			limit14 = 1;
		}		
	}
	
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
	{
		int pinStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		if (!pinStatus)
		{
			limsw15 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			limit15 = 0;
		}
		else if (pinStatus)
		{
			limsw15 = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			limit15 = 1;
		}		
	}

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
