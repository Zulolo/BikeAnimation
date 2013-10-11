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

	#define ACCE_COMMU_QUEUE_LEN				16
	#define ACCE_COMMU_BUSY_WAIT_MAX_US			20
	#define ACCE_COMMU_TIMEOUT_US				50
	#define ACCE_MONITOR_TIMER_PRESCAL			((SYSCLK_FREQ_72MHz / 1000000) - 1)		// Freerun TIM's frequency is at 1MHz (1us)
	#define ACCE_MONITOR_TIMER_PERIOD			1999	// 2ms repeat, for 5r/s, 100 sample per circle
	#define ACCE_ROUTINE_DATA_READ_ADDR			0x02
	#define ACCE_ROUTINE_DATA_READ_LEN			6
	#define ACCE_RECV_RAW_DATA_BUF_LEN			256		// 256x2ms = record for 0.5s, 
														// during this 0.5s there must be one max or one min.
														// Elsewise the LED will NOT be lite on
	#define NEW_ACCE_RECV_PV_NOT_FOUND_MAX		4		// If no new acce peak or valley recognized during 0.5x4 = 2s, turn off all LEDs
													
	#define ACCE_RECV_RAW_DATA_X_POS			1
	#define ACCE_RECV_RAW_DATA_Y_POS			3
	#define ACCE_RECV_RAW_DATA_Z_POS			5
	#define ACCE_ERROR_VALUE_INT16				0x8000
	#define GET_ACCE_RAW_DATA_FROM_BUF(pRXBuf)	((((uint16_t)(*(pRXBuf))) >> 6) | \
												(((uint16_t)((*((pRXBuf) + 1)) & 0x7F)) << 2) | \
												((((*((pRXBuf) + 1)) & 0x80) > 0)?(0x7E00):(0x0000))| \
												(((uint16_t)((*((pRXBuf) + 1)) & 0x80)) << 8))


	typedef enum {ACCE_CTRL_WR = 0, ACCE_CTRL_RD = !ACCE_CTRL_WR} ENUM_AcceCommuType;
	typedef enum {ACCE_WVF_V = 0, ACCE_WVF_P = !ACCE_WVF_V} ENUM_AccePeakValley;

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
		MOTION_STATE_RX_DATA_PROCCESS
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

	typedef struct
	{
		ENUM_AccePeakValley enumPeakValley;
		uint32_t iSysTickTime;
	} STR_AcceWVF_PV;

	static STR_AcceCommu staAcceCommuQueue[ACCE_COMMU_QUEUE_LEN];
	static DMA_InitTypeDef staAcceMasterTX_DMAInitStructure;
	static DMA_InitTypeDef staAcceMasterRX_DMAInitStructure;
	static uint8_t staAcceTXBUF[ACCE_TXRX_BUF_LEN_MAX];
	static uint8_t staAcceRXBUF[ACCE_TXRX_BUF_LEN_MAX];
	static ENUM_MotionDetectState staMotionDetectState = MOTION_STATE_IDLE; 
	static uint8_t staAcceCommuIndex = 0;
	static uint32_t staAcceCommuErrorCNT = 0;
	static int16_t staAcceRecvDawDataBufX_A[ACCE_RECV_RAW_DATA_BUF_LEN]; 
	static int16_t staAcceRecvDawDataBufX_B[ACCE_RECV_RAW_DATA_BUF_LEN]; 
	//static int16_t staAcceRecvDawDataBufZ[ACCE_RECV_RAW_DATA_BUF_LEN]; 
	static uint8_t staAcceRecvRawDataWalker = 0;
	static int32_t staAcceRecvRoundDataX;
	static int32_t staAcceRecvRoundDataY;
	static int32_t staAcceRecvRoundDataZ;
	static int16_t* pStaAcceRecvDawDataBufX;
	static uint16_t staPicRefreshInterval = PIC_REFRESH_ALL_LED_OFF;

	const uint8_t BMA020_CONFIG_PARA[] = {
		3, 0x14, 0x13 // +-8g, 188Hz
	};
#endif 


#endif
