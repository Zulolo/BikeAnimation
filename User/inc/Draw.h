#ifndef __DRAW_H
#define __DRAW_H

#include "GlobalVAR.h"
#include "PicFlash.h"


#define PIC_REFRESH_TIMER_CLK			RCC_APB1Periph_TIM2
#define LED_WR_PORT_CLK         		RCC_APB2Periph_GPIOB
#define LED_WR_PORT						GPIOB

#define LED_ALL_OFF						GPIO_Write(LED_WR_PORT, 0x00); \
										__NOP

#ifndef __MAIN_H		// means I am not included by Main.c
// Put all only Draw.c used declare here
// Main.c may use the defines above
// It's better to seperate models and put all the corresponding declare together

#endif 

#endif
