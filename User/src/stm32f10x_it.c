#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SysTick_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
//	GPIO_SetBits(LCD_BACKLIGHT_PORT, LCD_BACKLIGHT_PIN);
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
//	GPIO_SetBits(LCD_BACKLIGHT_PORT, LCD_BACKLIGHT_PIN);
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
//	GPIO_SetBits(LCD_BACKLIGHT_PORT, LCD_BACKLIGHT_PIN);
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	gSystem_1ms_CNT++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void ACCE_MONITOR_TIMER_IRQ_HANDLER(void)
{
	if (TIM_GetITStatus(ACCE_MONITOR_TIMER, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(ACCE_MONITOR_TIMER, TIM_IT_Update); 
		MTN_NewRoutineDataReadRequest();
	}
}

void PIC_REFRESH_TIMER_IRQ_HANDLER(void)
{
	// Max receiving time reached
	if (TIM_GetITStatus(PIC_REFRESH_TIMER, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(PIC_REFRESH_TIMER, TIM_IT_Update); 
	}
}

void ACCE_MASTER_DMA_TX_IRQHandler(void)
{
	uint16_t iTempSPIBusyWait = 0;
	if(DMA_GetITStatus(ACCE_MASTER_DMA_TX_IT_TC))
	{
		DMA_ClearITPendingBit(ACCE_MASTER_DMA_TX_IT_TC);
		
		// Check RXNE, TXE, BUSY first before Set NSS Pin
		// Here only check Busy flag because SPI trans length must larger than 1
		while (__IS_SPI_BUSY(ACCE_MASTER_SPI->SR) && (iTempSPIBusyWait < MAX_ACCE_TX_BUSY_WAIT_TIME))
		{
			iTempSPIBusyWait++;
		}

		ACCE_MASTER_NSS_DISABLE;
		DMA_ITConfig(ACCE_MASTER_DMA_TX_CHN, DMA_IT_TC, DISABLE);
		DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, DISABLE);
		FLAG_MOTION_ACCE_COMMU_BUSY = RESET;
	}	
}

void ACCE_MASTER_DMA_RX_IRQHandler(void)
{
	if(DMA_GetITStatus(ACCE_MASTER_DMA_RX_IT_TC))
	{
		DMA_ClearITPendingBit(ACCE_MASTER_DMA_RX_IT_TC);
		
		ACCE_MASTER_NSS_DISABLE;
		DMA_ITConfig(ACCE_MASTER_DMA_RX_CHN, DMA_IT_TC, DISABLE);
		DMA_Cmd(ACCE_MASTER_DMA_RX_CHN, DISABLE);
		FLAG_MOTION_ACCE_COMMU_BUSY = RESET;
	}	
}


 
 
 
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 
