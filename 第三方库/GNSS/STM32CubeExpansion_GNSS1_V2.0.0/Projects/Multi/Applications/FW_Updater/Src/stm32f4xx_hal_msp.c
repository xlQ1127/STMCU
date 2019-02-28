/**
  ******************************************************************************
  * @file    Applications/FW_Updater/Src/stm32f4xx_hal_msp.c
  * @author  AST/CL
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   This file provides code for the MSP Initialization 
  *          and de-Initialization codes.
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
#include "config_bus.h"

/* Private functions ---------------------------------------------------------*/

/* This function Initializes the Global MSP. */
void HAL_MspInit(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* This function is used for low level initialization of the UART peripheral. */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  if(huart->Instance==GNSS_UART_INSTANCE)
  {
    /* Peripheral clock enable */
    GNSS_UART_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GNSS_UART_RX|GNSS_UART_TX;
    GPIO_InitStruct.Mode = GNSS_UART_PIN_MODE;
    GPIO_InitStruct.Pull = GNSS_UART_PULL;
    GPIO_InitStruct.Speed = GNSS_UART_SPEED;
    GPIO_InitStruct.Alternate = GNSS_UART_ALTERNATE;
    HAL_GPIO_Init(GNSS_UART_RXTX_PORT, &GPIO_InitStruct);

    /* Configure the NVIC for USART1 */
    HAL_NVIC_SetPriority(GNSS_UART_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GNSS_UART_IRQn);

  }
  else if(huart->Instance==IO_UART_INSTANCE)
  {
    /* Peripheral clock enable */
    IO_UART_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = IO_UART_TX|IO_UART_RX;
    GPIO_InitStruct.Mode = IO_UART_PIN_MODE;
    GPIO_InitStruct.Pull = IO_UART_PULL;
    GPIO_InitStruct.Speed = IO_UART_SPEED;
    GPIO_InitStruct.Alternate = IO_UART_ALTERNATE;
    HAL_GPIO_Init(IO_UART_RXTX_PORT, &GPIO_InitStruct);
  }
}


/* This function is used for low level de-initialization of the UART peripheral. */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==GNSS_UART_INSTANCE)
  {
    /* Peripheral clock disable */
    GNSS_UART_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GNSS_UART_RXTX_PORT, GNSS_UART_TX|GNSS_UART_RX);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(GNSS_UART_IRQn);
  }
  else if(huart->Instance==IO_UART_INSTANCE)
  {
    /* Peripheral clock disable */
    IO_UART_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(IO_UART_RXTX_PORT, IO_UART_TX|IO_UART_RX);
  }

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
