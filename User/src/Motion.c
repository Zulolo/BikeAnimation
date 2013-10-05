
#include "Motion.h"

void ACCE_MasterConfiguration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;

	/* LCD_SPI_MASTER configuration ------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// Only TX
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		// Master of course
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// 8bit format
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		// High in idle
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	// Rising edge capture (TBD)
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	// 72M for APB2, 10M max reqirement for BMA020 SPI IF, so divide 8 = 9M for SPI CLK
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	// Accroding to BMA020's datasheet, first bit is MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(ACCE_MASTER_SPI, &SPI_InitStructure);

	/* SPI_DMA_Channel configuration ---------------------------------------------*/
	// Here only focus on some FIXED parameter, other parameter will be adjusted according to different event
	// like send command or fill rectangle...
	DMA_DeInit(ACCE_MASTER_DMA_TX_CHN);
	staAcceMasterTX_DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ACCE_MASTER_SPI->DR));
	staAcceMasterTX_DMAInitStructure.DMA_MemoryBaseAddr = (uint32_t)staAcceTXBUF;
	staAcceMasterTX_DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	staAcceMasterTX_DMAInitStructure.DMA_BufferSize = ACCE_TXRX_BUF_LEN_MAX;
	staAcceMasterTX_DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	staAcceMasterTX_DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	staAcceMasterTX_DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	staAcceMasterTX_DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	staAcceMasterTX_DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	staAcceMasterTX_DMAInitStructure.DMA_Priority = DMA_Priority_Medium;
	staAcceMasterTX_DMAInitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(ACCE_MASTER_DMA_TX_CHN, &staAcceMasterTX_DMAInitStructure);
	DMA_ITConfig(ACCE_MASTER_DMA_TX_CHN, DMA_IT_TC, DISABLE);

	DMA_DeInit(ACCE_MASTER_DMA_RX_CHN);
	staAcceMasterRX_DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ACCE_MASTER_SPI->DR));
	staAcceMasterRX_DMAInitStructure.DMA_MemoryBaseAddr = (uint32_t)staAcceRXBUF;
	staAcceMasterRX_DMAInitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	staAcceMasterRX_DMAInitStructure.DMA_BufferSize = ACCE_TXRX_BUF_LEN_MAX;
	staAcceMasterRX_DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	staAcceMasterRX_DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	staAcceMasterRX_DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	staAcceMasterRX_DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	staAcceMasterRX_DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	staAcceMasterRX_DMAInitStructure.DMA_Priority = DMA_Priority_Medium;
	staAcceMasterRX_DMAInitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(ACCE_MASTER_DMA_RX_CHN, &staAcceMasterRX_DMAInitStructure);
	DMA_ITConfig(ACCE_MASTER_DMA_RX_CHN, DMA_IT_TC, DISABLE);
	

	/* Enable SPI_MASTER NSS output for master mode */
	SPI_SSOutputCmd(ACCE_MASTER_SPI, ENABLE);

	/* Enable SPI_MASTER */
	SPI_Cmd(ACCE_MASTER_SPI, ENABLE);

	/* Enable Acce SPI's TX DMA */
	SPI_I2S_DMACmd(ACCE_MASTER_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(ACCE_MASTER_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable DMA1 Channel3 */
	//DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, ENABLE);
}

ENUM_OperationResult AcceReadRX_DMAInitial(uint8_t iBUFLength, uint8_t* pRecvContainer)
{
	DMA_Cmd(ACCE_MASTER_DMA_RX_CHN, DISABLE);
	DMA_ITConfig(ACCE_MASTER_DMA_RX_CHN, DMA_IT_TC, DISABLE);
	staAcceMasterRX_DMAInitStructure.DMA_MemoryBaseAddr = (uint32_t)pRecvContainer;
	staAcceMasterRX_DMAInitStructure.DMA_BufferSize = iBUFLength;
	DMA_Init(ACCE_MASTER_DMA_RX_CHN, &staAcceMasterRX_DMAInitStructure);
	return ACCEPTED;
}

ENUM_OperationResult AcceReadTX_DMAInitial(uint8_t iRegAddress, uint8_t iBUFLength)
{
	DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, DISABLE);
	DMA_ITConfig(ACCE_MASTER_DMA_TX_CHN, DMA_IT_TC, DISABLE);
	staAcceTXBUF[0] = iRegAddress | MSB_MASK_8BIT;
	staAcceMasterRX_DMAInitStructure.DMA_MemoryBaseAddr = (uint32_t)staAcceTXBUF;
	staAcceMasterTX_DMAInitStructure.DMA_BufferSize = iBUFLength;
	DMA_Init(ACCE_MASTER_DMA_TX_CHN, &staAcceMasterTX_DMAInitStructure);
	return ACCEPTED;
}

ENUM_OperationResult AcceWriteTX_DMAInitial(uint8_t* pTXBuffer, uint8_t iBUFLength)
{
	DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, DISABLE);
	DMA_Cmd(ACCE_MASTER_DMA_RX_CHN, DISABLE);
	DMA_ITConfig(ACCE_MASTER_DMA_TX_CHN, DMA_IT_TC, DISABLE);
	DMA_ITConfig(ACCE_MASTER_DMA_RX_CHN, DMA_IT_TC, DISABLE);
	staAcceMasterRX_DMAInitStructure.DMA_MemoryBaseAddr = (uint32_t)pTXBuffer;
	staAcceMasterTX_DMAInitStructure.DMA_BufferSize = iBUFLength;
	DMA_Init(ACCE_MASTER_DMA_TX_CHN, &staAcceMasterTX_DMAInitStructure);
	return ACCEPTED;
}

ENUM_OperationResult AcceRead(uint8_t iRegAddress, uint8_t iReadLength, uint8_t* pRecvContainer)
{
	AcceReadRX_DMAInitial(iReadLength + 1, pRecvContainer);
	AcceReadTX_DMAInitial(iRegAddress, iReadLength + 1);
	DMA_ITConfig(ACCE_MASTER_DMA_RX_CHN, DMA_IT_TC, ENABLE);
	ACCE_MASTER_NSS_ENABLE;
	FLAG_MOTION_ACCE_COMMU_BUSY = SET;
	DMA_Cmd(ACCE_MASTER_DMA_RX_CHN, ENABLE);
	DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, ENABLE);
	return ACCEPTED;
}

ENUM_OperationResult AcceWrite(uint8_t* pTXBuffer, uint8_t iWriteLength)
{	
	AcceWriteTX_DMAInitial(pTXBuffer, iWriteLength);
	DMA_ITConfig(ACCE_MASTER_DMA_TX_CHN, DMA_IT_TC, ENABLE);
	ACCE_MASTER_NSS_ENABLE;
	FLAG_MOTION_ACCE_COMMU_BUSY = SET;
	DMA_Cmd(ACCE_MASTER_DMA_TX_CHN, ENABLE);
	return ACCEPTED;
}

STR_AcceCommu* AcceQueueSlotSearch(STR_AcceCommu* pAcceCommuQueueHead, 
								  uint8_t iAcceCommuQueueLen, 
								  uint8_t iAcceCommuWorkingPos)
{
	uint8_t temp_QueueIndex;
	STR_AcceCommu* pAcceCommuCurrent = pAcceCommuQueueHead + iAcceCommuWorkingPos;
	if ((NULL == (*pAcceCommuCurrent).pRXBuf) &&
		(NULL == (*pAcceCommuCurrent).pTXBuf))
	{
		return pAcceCommuCurrent;
	}
	else
	{
		temp_QueueIndex = iAcceCommuWorkingPos + 1;
		while (temp_QueueIndex < iAcceCommuQueueLen)
		{
			pAcceCommuCurrent = pAcceCommuQueueHead + temp_QueueIndex;
			if ((NULL == (*pAcceCommuCurrent).pRXBuf) &&
				(NULL == (*pAcceCommuCurrent).pTXBuf))
			{
				return pAcceCommuCurrent;
			}
			temp_QueueIndex++;
		}
		temp_QueueIndex = 0;
		while (temp_QueueIndex < iAcceCommuWorkingPos)
		{
			pAcceCommuCurrent = pAcceCommuQueueHead + temp_QueueIndex;
			if ((NULL == (*pAcceCommuCurrent).pRXBuf) &&
				(NULL == (*pAcceCommuCurrent).pTXBuf))
			{
				return pAcceCommuCurrent;
			}
			temp_QueueIndex++;
		}
		return NULL;
	}
}

void AcceQueueSlotFree(STR_AcceCommu *pAcceQueueSlot)
{
	// Only free the space when they are in heap
	// May be const value like start up device config parameter stream
	if (NULL != pAcceQueueSlot->pRXBuf)
	{
		if (__IS_IN_HEAP(pAcceQueueSlot->pRXBuf))
		{
			free(pAcceQueueSlot->pRXBuf);
		}
		pAcceQueueSlot->pRXBuf = NULL;
	}
	if (NULL != pAcceQueueSlot->pTXBuf)
	{
		if (__IS_IN_HEAP(pAcceQueueSlot->pTXBuf))
		{
			free(pAcceQueueSlot->pTXBuf);
		}
		pAcceQueueSlot->pTXBuf = NULL;
	}
}

// Better to invoke AcceQueueSlotInitial() and HeapInitial() together
void AcceQueueSlotInitial(void)
{
	uint8_t tempIndex;
	staAcceCommuIndex = 0;	// Initial staAcceCommuIndex in acce commu queue to 0
	for (tempIndex = 0; tempIndex < ACCE_COMMU_QUEUE_LEN; tempIndex++)
	{
		(staAcceCommuQueue + tempIndex)->pRXBuf = NULL;
		(staAcceCommuQueue + tempIndex)->pTXBuf = NULL;
	}
}

void HeapInitial(void)
{
	memset((uint8_t *)HEAP_BASE_SCT, NULL, HEAP_SIZE);
}

void MotionStateInitial(void)
{
	staMotionDetectState = MOTION_STATE_MASTER_CONF;
}

ENUM_OperationResult AcceMasterBlastTX_Const(const uint8_t* pFormattedCMDStream, uint8_t iFormattedCMDStreamLen)
{
	uint8_t iTempIndex = 0;
	STR_AcceCommu* pTempAvailableQueueSlot = NULL;
	while (iTempIndex < iFormattedCMDStreamLen)
	{
		if (2 > *(pFormattedCMDStream + iTempIndex))
		{
			return REFUSED;
		}
		pTempAvailableQueueSlot = AcceQueueSlotSearch(staAcceCommuQueue, sizeof(staAcceCommuQueue), staAcceCommuIndex);
		if (NULL == pTempAvailableQueueSlot)
		{
			return REFUSED;
		}
		pTempAvailableQueueSlot->pRXBuf = NULL;
		pTempAvailableQueueSlot->bCommuSuccess = FALSE;
		pTempAvailableQueueSlot->pTXBuf = (uint8_t *)(pFormattedCMDStream + iTempIndex + 1);
		pTempAvailableQueueSlot->iTXBufLen = *(pFormattedCMDStream + iTempIndex) - 1;
		iTempIndex += *(pFormattedCMDStream + iTempIndex);
	}
	return ACCEPTED;
}

void AcceDeviceConfig(void)
{
/* Like this, the first number of each line is the number of bytes
   (Include it self)
	const uint8_t BMA020_CONFIG_PARA[] = {
		6, 0x00, 0x01, 0x02, 0x03, 0x04,
		3, 0x05, 0x12,
		4, 0x04, 0x17, 0x28
	}
*/
	AcceMasterBlastTX_Const(BMA020_CONFIG_PARA, sizeof(BMA020_CONFIG_PARA));
	staMotionDetectState = MOTION_STATE_TASK_DISPATCH;
}

void MotionManager(void)
{
	static uint16_t staAcceCommuBusyWaitTime;
	switch (staMotionDetectState)
	{
	case MOTION_STATE_IDLE:

		break;

	case MOTION_STATE_MASTER_CONF:
			/* MCU periphery config ------------------------------------------------------*/
			ACCE_MasterConfiguration();
			staMotionDetectState = MOTION_STATE_MASTER_INIT;
		break;

	case MOTION_STATE_MASTER_INIT:
			/* MCU queue slot and heap initial ------------------------------------------------------*/
			AcceQueueSlotInitial();
			HeapInitial();
			staMotionDetectState = MOTION_STATE_SLAVE_CONF;

		break;

	case MOTION_STATE_SLAVE_CONF:
			AcceDeviceConfig();
			// staMotionDetectState was changed in the routine of AcceDeviceConfig() because some device may need other state
		break;

	case MOTION_STATE_TASK_DISPATCH:
			if (NULL != staAcceCommuQueue[staAcceCommuIndex].pRXBuf)
			{
				staAcceCommuBusyWaitTime = GET_GLOBAL_FREERUN_US;
				staMotionDetectState = MOTION_STATE_TRIGGER_RX;
			}
			else if (NULL != staAcceCommuQueue[staAcceCommuIndex].pTXBuf)
			{
				staAcceCommuBusyWaitTime = GET_GLOBAL_FREERUN_US;
				staMotionDetectState = MOTION_STATE_TRIGGER_TX;
			}
			else
			{
				//staMotionDetectState = MOTION_STATE_IDLE;
			}

		break;

	case MOTION_STATE_TRIGGER_TX:
		while (FLAG_MOTION_ACCE_COMMU_BUSY && GLOBAL_FREERUN_US_WAITING(staAcceCommuBusyWaitTime, ACCE_COMMU_BUSY_WAIT_MAX_US))	
		{	// Normally FLAG_MOTION_ACCE_COMMU_BUSY will only be busy in MOTION_STATE_TXING or MOTION_STATE_RXING state
		}
		if (FLAG_MOTION_ACCE_COMMU_BUSY)
		{
			FLAG_MOTION_ACCE_COMMU_BUSY = RESET; // Force reset of FLAG_MOTION_ACCE_COMMU_BUSY
			staAcceCommuErrorCNT++;
		}
		AcceWrite(staAcceCommuQueue[staAcceCommuIndex].pTXBuf, 
				 staAcceCommuQueue[staAcceCommuIndex].iTXBufLen);
		staAcceCommuBusyWaitTime = GET_GLOBAL_FREERUN_US;
		staMotionDetectState = MOTION_STATE_TXING;
		break;

	case MOTION_STATE_TXING:
		if (RESET == FLAG_MOTION_ACCE_COMMU_BUSY)
		{
			AcceQueueSlotFree(staAcceCommuQueue + staAcceCommuIndex);
			ACCE_QUEUE_ROAMER_FORWARD;
			staMotionDetectState = MOTION_STATE_TASK_DISPATCH;
		}
		else if (GLOBAL_FREERUN_US_TIMEOUT(staAcceCommuBusyWaitTime, ACCE_COMMU_TIMEOUT_US))
		{
			AcceQueueSlotFree(staAcceCommuQueue + staAcceCommuIndex);
			ACCE_QUEUE_ROAMER_FORWARD;
			staAcceCommuErrorCNT++;
			staMotionDetectState = MOTION_STATE_TASK_DISPATCH;
		}
		break;

	case MOTION_STATE_TRIGGER_RX:
		while (FLAG_MOTION_ACCE_COMMU_BUSY && GLOBAL_FREERUN_US_WAITING(staAcceCommuBusyWaitTime, ACCE_COMMU_BUSY_WAIT_MAX_US))	
		{	// Normally FLAG_MOTION_ACCE_COMMU_BUSY will only be busy in MOTION_STATE_TXING or MOTION_STATE_RXING state
		}
		if (FLAG_MOTION_ACCE_COMMU_BUSY)
		{
			FLAG_MOTION_ACCE_COMMU_BUSY = RESET; // Force reset of FLAG_MOTION_ACCE_COMMU_BUSY
			staAcceCommuErrorCNT++;
		}
		AcceRead(staAcceCommuQueue[staAcceCommuIndex].iRXAddress, 
				 staAcceCommuQueue[staAcceCommuIndex].iRXBufLen - 1,
				 staAcceCommuQueue[staAcceCommuIndex].pRXBuf);
		staAcceCommuBusyWaitTime = GET_GLOBAL_FREERUN_US;
		staMotionDetectState = MOTION_STATE_RXING;
		break;

	case MOTION_STATE_RXING:
		if (RESET == FLAG_MOTION_ACCE_COMMU_BUSY)
		{
			staAcceCommuQueue[staAcceCommuIndex].bCommuSuccess = TRUE;
			staMotionDetectState = MOTION_STATE_RX_DATA_PROCCESS;
		}
		else if (GLOBAL_FREERUN_US_TIMEOUT(staAcceCommuBusyWaitTime, ACCE_COMMU_TIMEOUT_US))
		{
			staAcceCommuQueue[staAcceCommuIndex].bCommuSuccess = FALSE;
			staAcceCommuErrorCNT++;
			staMotionDetectState = MOTION_STATE_RX_DATA_PROCCESS;
		}

		break;

	case MOTION_STATE_RX_DATA_PROCCESS:
		if (TRUE == staAcceCommuQueue[staAcceCommuIndex].bCommuSuccess)
		{

		}
		else
		{
			
		}
		AcceQueueSlotFree(staAcceCommuQueue + staAcceCommuIndex);
		ACCE_QUEUE_ROAMER_FORWARD;
		staMotionDetectState = MOTION_STATE_TASK_DISPATCH;
		break;

		default :
		break;
	}
}
