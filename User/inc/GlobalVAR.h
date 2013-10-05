#ifndef __GLOBAL_VAR_H
#define __GLOBAL_VAR_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#ifdef __VAR_DEFINE
	#define __GVAR
#else 
	#define __GVAR		extern
#endif

#define NULL 			0

#define SYSCLK_FREQ_72MHz               	72000000
#define APB1CLK_FREQ_36MHz  		        36000000

#define ACCE_MASTER_PORT				    GPIOA
#define ACCE_MASTER_NSS_PIN             	GPIO_Pin_4
#define ACCE_MASTER_SCK_PIN             	GPIO_Pin_5
#define ACCE_MASTER_MISO_PIN          		GPIO_Pin_6
#define ACCE_MASTER_MOSI_PIN            	GPIO_Pin_7
#define ACCE_MASTER_SPI          			SPI1
#define ACCE_MASTER_NSS_DISABLE    			GPIO_SetBits(ACCE_MASTER_PORT, ACCE_MASTER_NSS_PIN)
#define ACCE_MASTER_NSS_ENABLE    			GPIO_ResetBits(ACCE_MASTER_PORT, ACCE_MASTER_NSS_PIN)

#define ACCE_MASTER_DMA_TX_IRQn			    DMA1_Channel3_IRQn
#define ACCE_MASTER_DMA_TX_IRQHandler	    DMA1_Channel3_IRQHandler
#define ACCE_MASTER_DMA_TX_IT_TC		    DMA1_IT_TC3
#define ACCE_MASTER_DMA_TX_CHN			    DMA1_Channel3	// SPI1_TX DMA
#define ACCE_MASTER_DMA_RX_IRQn			    DMA1_Channel2_IRQn
#define ACCE_MASTER_DMA_RX_IRQHandler	  	DMA1_Channel2_IRQHandler
#define ACCE_MASTER_DMA_RX_IT_TC		    DMA1_IT_TC2
#define ACCE_MASTER_DMA_RX_CHN			    DMA1_Channel2	// SPI1_RX DMA
#define PIC_REFRESH_TIMER_IRQn			    TIM2_IRQn
#define SIG_GEN_TIMER 				        TIM1
#define PIC_REFRESH_TIMER 				    TIM2
#define GLB_FREERUN_TIMER 				    TIM3    // Without interrupt, evryone need to use this, divided into 1us per CNT

#define LSB_MASK_8BIT					    0x01
#define LSB_FILTER_8BIT					    0xFE
#define MSB_MASK_8BIT					    0x80
#define MSB_FILTER_8BIT					    0x7F

typedef enum {FALSE = 0, TRUE = !FALSE} Boolean;
typedef enum {REFUSED = 0, ACCEPTED = !REFUSED} ENUM_OperationResult;
typedef enum {FREE = 0, BUSY = !FREE} ENUM_BusStatus;

#define HEAP_BASE_SCT						0x20003000		// According to .sct file
#define HEAP_SIZE							0x200
#define __IS_IN_HEAP(x)                 	(((uint32_t)(x) >= HEAP_BASE_SCT) ? TRUE : FALSE)

//Used to find the BB start address of a 32bits variable
#define BITBAND_ADDRESS(x)			        (((x) & 0xF0000000) + 0x02000000 + (((x) & 0xFFFFF) << 5))	
//Used to find the bit's BB address of a 32bits variable given address				
#define BITBAND(address,bit)		        (*((volatile uint32_t*)(BITBAND_ADDRESS((uint32_t)(address)) + ((bit) << 2))))
//Used to find the bit's BB address of a 32bits variable given variable	
#define BITBAND_V(variable,bit)		      (*((volatile uint32_t*)(BITBAND_ADDRESS((uint32_t)(&(variable))) + ((bit) << 2))))

__GVAR uint32_t gSystem_1ms_CNT;

__GVAR uint32_t iFlagGlobalGeneralG01;
#define FLAG_HEAP_STATUS						BITBAND_V(iFlagGlobalGeneralG01, 0)	 

__GVAR uint32_t iFlagMotionGeneralG01;
#define FLAG_MOTION_ACCE_COMMU_BUSY				BITBAND_V(iFlagMotionGeneralG01, 0)	

__GVAR uint32_t iFlagUSBGeneralG01;		   	

__GVAR uint32_t iFlagDrawGeneralG01;		  

__GVAR uint32_t iFlagGlobalError;		   	

__GVAR void Occupy_Delay(uint32_t delayMs);

extern void memor(void *dest, const void *src, uint32_t size);
extern void ALL_LED_OFF(void);

__GVAR EXTI_InitTypeDef   gEXTI_InitStructure;

#define GET_GLOBAL_FREERUN_US   				TIM_GetCounter(GLB_FREERUN_TIMER)
#define GLOBAL_FREERUN_US_WAITING(iStart, iMax)	(((uint16_t)((GET_GLOBAL_FREERUN_US) - (iStart))) < (iMax))		

#endif
