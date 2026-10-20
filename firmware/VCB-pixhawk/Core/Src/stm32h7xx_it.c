/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  *          VCB pixhawk-pwm build: USART2 (CRSF) removed, TIM2/TIM5 (PWM
  *          input capture) added. Based on cttdev/VCB.
  ******************************************************************************
  */
#include "main.h"
#include "stm32h7xx_it.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
   while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/** DMA1 stream1: UART4 TX (console printf). */
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}

/** TIM2: PWM input ch2 (PA2, left motor). */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

/** TIM5: PWM input ch1 (PA3, right motor). */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

/** UART4: console. */
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}
