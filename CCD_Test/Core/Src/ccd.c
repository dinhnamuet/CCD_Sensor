/*
 * ccd.c
 *
 *  Created on: Aug 26, 2024
 *      Author: dinhnamuet
 */
#include "ccd.h"
#include <math.h>

static void ccd_reset(struct ccd *ccd_s) {
	HAL_TIM_PWM_Stop(ccd_s->M, TIM_CHANNEL_1); 	/* phi M */
	HAL_TIM_PWM_Stop(ccd_s->ICG, TIM_CHANNEL_1); 	/* ICG */
	HAL_GPIO_WritePin(ccd_s->SH.port, ccd_s->SH.pin_no, GPIO_PIN_RESET);

	__HAL_TIM_SET_COUNTER(ccd_s->M, 0);
	__HAL_TIM_SET_COUNTER(ccd_s->ICG, 0);
}

int ccd_init(struct ccd *ccd_s, TIM_HandleTypeDef *icg_pin, TIM_HandleTypeDef *m_pin, GPIO_TypeDef *sh_port, uint16_t sh_pin_no) {
	if (!ccd_s || !icg_pin || !m_pin || !sh_port) {
		return -1;
	} else {
		ccd_s->SH.port = sh_port;
		ccd_s->SH.pin_no = sh_pin_no;
		ccd_s->ICG = icg_pin;
		ccd_s->M = m_pin;
		ccd_reset(ccd_s);
		ccd_s->ICG->Instance->CCR2 = ((T2*8/100 - 1) - ISR_LATENC_PULSE) - T2_PROCESS_PULSE; /* T2 500ns, set SH high level on compare match occurs */
		ccd_s->ICG->Instance->CCR3 = ((T2*8/100 - 1) + (T3*8/100) - ISR_LATENC_PULSE) - T3_PROCESS_PULSE; /* T3 4000ns, set SH Low level on compare match occurs */
		ccd_s->ICG->Instance->CCR4 = ccd_s->ICG->Instance->CCR1 - ((ccd_s->M->Instance->PSC + 1) * round((float)ccd_s->M->Instance->CCR1/2));
		ccd_s->M->Instance->CCR2 = ccd_s->M->Instance->CCR1 - ISR_LATENC_PULSE - TIM1_CC_PROCESS_PULSE;
		__HAL_TIM_ENABLE_IT(ccd_s->ICG, TIM_IT_CC2);
		__HAL_TIM_ENABLE_IT(ccd_s->ICG, TIM_IT_CC3);
		__HAL_TIM_ENABLE_IT(ccd_s->M, TIM_IT_CC2);
	}
	return 0;
}

HAL_StatusTypeDef set_integration_time(struct ccd *ccd_s, uint32_t us) {
	double foo = 0;
	/* Integration time min 3102 us */
	if (us < 3102) {
		return HAL_ERROR;
	}

	ccd_reset(ccd_s);
	foo = (double)us*80/(double)(ccd_s->ICG->Instance->PSC + 1);

	ccd_s->ICG->Instance->ARR = (uint32_t)foo - 1;

	return HAL_OK;
}

uint32_t get_integration_time_us(struct ccd *ccd_s) {
	return ccd_s->ICG->Instance->ARR/80;
}

void get_spectrum(struct ccd *ccd_s) {
	HAL_TIM_PWM_Start(ccd_s->M, TIM_CHANNEL_1); 	/* phi M */
	HAL_TIM_PWM_Start(ccd_s->ICG, TIM_CHANNEL_1); 	/* ICG */
}
