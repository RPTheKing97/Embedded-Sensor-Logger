/* =================== INCLUDES =================== */
#include "stm32f4xx_hal.h"

/* =================== EXTERNS =================== */
extern TIM_HandleTypeDef htim2;

/* =================== CPU EXCEPTIONS =================== */
void NMI_Handler(void)               { while (1) {} }
void HardFault_Handler(void)         { while (1) {} }
void MemManage_Handler(void)         { while (1) {} }
void BusFault_Handler(void)          { while (1) {} }
void UsageFault_Handler(void)        { while (1) {} }
void SVC_Handler(void)               { }
void DebugMon_Handler(void)          { }
void PendSV_Handler(void)            { }

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/* =================== IRQ HANDLERS =================== */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}
