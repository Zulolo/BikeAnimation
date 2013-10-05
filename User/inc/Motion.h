#ifndef __MOTION_H
#define __MOTION_H

#include "GlobalVAR.h"
#include <math.h>


#define ACCE_MASTER_DMA_CLK				RCC_AHBPeriph_DMA1
#define ACCE_MASTER_CLK              	RCC_APB2Periph_SPI1
#define ACCE_MASTER_PORT_CLK         	RCC_APB2Periph_GPIOA 
#define ACCE_TXRX_BUF_LEN_MAX			32
#define ACCE_TXRX_BUF_LEN_MAX			32
#define ACCE_MASTER_TX_DR_ADD    		0x4000300C
#define ACCE_MASTER_RX_DR_ADD    		0x4000300C

#ifndef __MAIN_H		// means I am not included by Main.c
// Put all only Motion.c used declare here
// Main.c may use the defines above
// It's better to seperate models and put all the corresponding declare together

	#define ACCE_COMMU_QUEUE_LEN			16
	#define ACCE_COMMU_BUSY_WAIT_MAX_US		20
	#define ACCE_COMMU_TIMEOUT_US			50

	typedef enum {ACCE_CTRL_WR = 0, ACCE_CTRL_RD = !ACCE_CTRL_WR} ENUM_AcceCommuType;

	typedef enum {	
		MOTION_STATE_IDLE = 0,
		MOTION_STATE_MASTER_CONF,	// MCU periphery config
		MOTION_STATE_MASTER_INIT,	// MCU queue slot and heap initial
		MOTION_STATE_SLAVE_CONF,	// Acce parameter config
		MOTION_STATE_TASK_DISPATCH,	// Dispatch task to TX or RX trigger from task queue
		MOTION_STATE_TRIGGER_TX,	// 
		MOTION_STATE_TXING,
		MOTION_STATE_TRIGGER_RX,
		MOTION_STATE_RXING,
		MOTION_STATE_DATA_PROCCESS
	} ENUM_MotionDetectState;

	typedef struct
	{
		uint8_t* pTXBuf;
		uint8_t iTXBufLen;
		uint8_t iRXAddress;
		uint8_t* pRXBuf;
		uint8_t iRXBufLen;
		Boolean bCommuSuccess;
	} STR_AcceCommu;

	static STR_AcceCommu staAcceCommuQueue[ACCE_COMMU_QUEUE_LEN];
	static DMA_InitTypeDef staAcceMasterTX_DMAInitStructure;
	static DMA_InitTypeDef staAcceMasterRX_DMAInitStructure;
	static uint8_t staAcceTXBUF[ACCE_TXRX_BUF_LEN_MAX];
	static uint8_t staAcceRXBUF[ACCE_TXRX_BUF_LEN_MAX];
	static ENUM_MotionDetectState staMotionDetectState = MOTION_STATE_IDLE; 
	static uint8_t staAcceCommuIndex = 0;
	static uint32_t staAcceCommuErrorCNT = 0;

	const uint8_t BMA020_CONFIG_PARA[] = {
		6, 0x00, 0x01, 0x02, 0x03, 0x04,
		3, 0x05, 0x12,
		4, 0x04, 0x17, 0x28
	};
#endif 


#endif
