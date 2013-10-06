
/* Includes ------------------------------------------------------------------*/
#include "main.h"
 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

	// Config system 1ms clock
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		// Do some thing
		while (1);
	}
		     
	/* System clocks configuration ---------------------------------------------*/
	RCC_Configuration();
	
	/* Interrupt configuration ---------------------------------------------*/
	NVIC_Configuration();

	/* GPIO configuration ------------------------------------------------------*/
	GPIO_Configuration();

	/* Free run TIM configuration ------------------------------------------------------*/
	GLB_FreerunTIM_Initial();
	
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
		/* Clear reset flags */
		RCC_ClearFlag();
	}
	else
	{

	}
	IWDG_Configuration();

	MotionStateInitial();

	while (1)
	{
		IWDG_ReloadCounter();  
		USBManager();
		MotionManager();
		RedrawManager();
	}
}


void Occupy_Delay(uint32_t delayMs)
{
	uint32_t entrTime = gSystem_1ms_CNT;
	while ((uint32_t)(gSystem_1ms_CNT - entrTime) < delayMs)
	{
	}
}

void IWDG_Configuration(void)
{
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE); 
	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
	dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
	Counter Reload Value = 250ms/IWDG counter clock period
					  = 0.25s / (1 / (LsiFreq/32))
					  = LsiFreq/(32 * 4)
	    			  = LsiFreq/128
	Counter Reload Value = 100ms/IWDG counter clock period
					  = 0.1s / (1 / (LsiFreq/32))
					  = LsiFreq/(32 * 10)
					  = LsiFreq/320
	*/
	IWDG_SetReload(LSI_FREQUENCY / 320);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);	// 72M for APB2, 10M max requirement for Acce and Gyro, so divide 32 = 2.25M for SPI CLK
	RCC_PCLK1Config(RCC_HCLK_Div2);	// For TIM3

	/* Enable peripheral clocks --------------------------------------------------*/
	/* Enable Acce SPI DMA clock */
	RCC_AHBPeriphClockCmd(ACCE_MASTER_DMA_CLK, ENABLE);

	/* Enable clock for Acce SPI_MASTER GPIO and other controls on APB2 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
						   ACCE_MASTER_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable clock for TIM4 of SIG_IN, ... */
	RCC_APB1PeriphClockCmd(PIC_REFRESH_TIMER_CLK | GLB_FREERUN_TIMER_CLK, ENABLE);
                      
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Port B General
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
									GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
									GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
									GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LED_WR_PORT, &GPIO_InitStructure);
	LED_ALL_OFF;	// Pull down GPIOB

	/* Configure Acce SPI_MASTER pins: SCK, MOSI and NSS */
	GPIO_InitStructure.GPIO_Pin = ACCE_MASTER_SCK_PIN | ACCE_MASTER_MOSI_PIN;	// | ACCE_MASTER_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(ACCE_MASTER_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ACCE_MASTER_NSS_PIN;	// | ACCE_MASTER_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(ACCE_MASTER_PORT, &GPIO_InitStructure);
	ACCE_MASTER_NSS_DISABLE;

	/* Configure Acce SPI_MASTER pins: SCK, MOSI and NSS */
	GPIO_InitStructure.GPIO_Pin = ACCE_MASTER_MISO_PIN;	// | ACCE_MASTER_NSS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(ACCE_MASTER_PORT, &GPIO_InitStructure);

}

void NVIC_Configuration(void)
{
// local variables 
   NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
  /* Configure the preemption priority and subpriority:
     - 1 bits for pre-emption priority: possible value are 0 or 1 
     - 3 bits for subpriority: possible value are 0..7
     - Lower values gives higher priority  
   */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
	NVIC_InitStructure.NVIC_IRQChannel = ACCE_MASTER_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = ACCE_MASTER_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM4_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = PIC_REFRESH_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM2_IRQn global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ACCE_MONITER_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

} // end NVIC_Configuration

void GLB_FreerunTIM_Initial(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = GLB_FREERUN_TIMER_PRESCAL;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = GLB_FREERUN_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(GLB_FREERUN_TIMER, &TIM_TimeBaseStructure);
	TIM_Cmd(GLB_FREERUN_TIMER, ENABLE);
}

void memor(void *dest, const void *src, uint32_t size)
{
	register uint32_t temp_i = size;
	while (temp_i)
	{
		*((uint8_t *)dest) = *((uint8_t *)dest) | *((uint8_t *)src);
		temp_i--;
	}
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2013 Zulolo *****END OF FILE****/

