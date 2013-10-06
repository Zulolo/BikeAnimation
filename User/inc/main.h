#ifndef __MAIN_H
#define __MAIN_H

#define __VAR_DEFINE
#include "GlobalVAR.h"
#include "Draw.h"
#include "Motion.h"

#define LSI_FREQUENCY					40000

#define GLB_FREERUN_TIMER_PRESCAL		((SYSCLK_FREQ_72MHz / 1000000) - 1)		// Freerun TIM's frequency is at 1MHz (1us)
#define GLB_FREERUN_TIMER_PERIOD		65535	// 0.000001s * 65530 * 1531 / 2 = 50.163m MAX repeat

#define ACCE_MONITER_TIMER_CLK			RCC_APB1Periph_TIM2
#define GLB_FREERUN_TIMER_CLK			RCC_APB1Periph_TIM3
#define PIC_REFRESH_TIMER_CLK			RCC_APB1Periph_TIM4

void RCC_Configuration(void);
void GPIO_Configuration(void);
void IWDG_Configuration(void);
void GLB_FreerunTIM_Initial(void);
//void COMMU_Manager(void);
void NVIC_Configuration(void);

extern void USBManager(void);
extern void MotionManager(void);
extern void RedrawManager(void);

extern void ACCE_MasterConfiguration(void);
extern ENUM_OperationResult AcceRead(uint8_t iRegAddress, uint8_t iRegLength, uint8_t* pRecvContainer);
extern void MotionStateInitial(void);

#endif
