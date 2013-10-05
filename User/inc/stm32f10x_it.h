#ifndef __STM32F10X_IT_H
#define __STM32F10X_IT_H

#include "GlobalVAR.h"
#include <math.h>

#define PIC_REFRESH_TIMER_IRQ_HANDLER			TIM2_IRQHandler
#define __IS_SPI_BUSY(x)						(((x) & SPI_I2S_FLAG_BSY) > 0)
#define MAX_ACCE_TX_BUSY_WAIT_TIME				4096

#endif
