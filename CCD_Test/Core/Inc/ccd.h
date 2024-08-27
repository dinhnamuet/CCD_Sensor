/*
 * ccd.h
 *
 *  Created on: Aug 26, 2024
 *      Author: dinhnamuet
 */

#ifndef INC_CCD_H_
#define INC_CCD_H_

#include "stm32l4xx_hal.h"

/* Timing in nanoseconds */
#define T1	6000	/* ICG pulse delay */
#define T2	500 	/* Pulse timing of ICG and SH */
#define T3	4000	/* Shift pulse width */
#define t4	40		/* Pulse timing of ICG and M */

#define ISR_LATENC_PULSE	12 /* Stack frame store and get ISR from the vector table */
#define T2_PROCESS_PULSE	27 /* 27 assembly command */
#define T3_PROCESS_PULSE	35 /* 35 assembly command */

#define TIM1_CC_PROCESS_PULSE	30

#pragma pack(1)
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin_no;
} gpio_pin_t;

struct ccd {
	gpio_pin_t SH;
	TIM_HandleTypeDef *ICG;
	TIM_HandleTypeDef *M;
};
#pragma pack()

int ccd_init(struct ccd *ccd_s, TIM_HandleTypeDef *icg_pin, TIM_HandleTypeDef *m_pin, GPIO_TypeDef *sh_port, uint16_t sh_pin_no);
HAL_StatusTypeDef set_integration_time(struct ccd *ccd_s, uint32_t us);
uint32_t get_integration_time_us(struct ccd *ccd_s);
void get_spectrum(struct ccd *ccd_s);

#endif /* INC_CCD_H_ */
