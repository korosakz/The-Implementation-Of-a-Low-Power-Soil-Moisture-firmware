/**
  ******************************************************************************
  * File Name          : LPTIM.c
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lptim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t lptim_dbg_flg;
/* USER CODE END 0 */

/* LPTIM1 init function */
void MX_LPTIM1_Init(void){

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

	/**LPTIM1 GPIO Configuration
	PA8  ------> LPTIM2_OUT
	*/

	/* LPTIM1 interrupt Init */
	NVIC_SetPriority(LPTIM1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(LPTIM1_IRQn);

	LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
	LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV2);  //period 61us
	LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
	LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
	LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
	//LL_LPTIM_TrigSw(LPTIM1);
	LL_LPTIM_EnableIT_CMPM(LPTIM1);

}

void MX_LPTIM2_Init(void){

	LL_GPIO_InitTypeDef GPIO_InitStruct;
	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPTIM2);

	/**LPTIM1 GPIO Configuration
	PA8  ------> LPTIM2_OUT
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* LPTIM1 interrupt Init */
	NVIC_SetPriority(LPTIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(LPTIM2_IRQn);

	LL_LPTIM_SetClockSource(LPTIM2, LL_LPTIM_CLK_SOURCE_INTERNAL);
	LL_LPTIM_SetPrescaler(LPTIM2, LL_LPTIM_PRESCALER_DIV1);
	LL_LPTIM_ConfigOutput(LPTIM2, LL_LPTIM_OUTPUT_WAVEFORM_PWM, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
	LL_LPTIM_SetUpdateMode(LPTIM2, LL_LPTIM_UPDATE_MODE_ENDOFPERIOD);
	LL_LPTIM_SetCounterMode(LPTIM2, LL_LPTIM_COUNTER_MODE_INTERNAL);

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_SetCompare(LPTIM2, 0x1);
	LL_LPTIM_SetAutoReload(LPTIM2, 3);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

}

void lptim_set_divider(uint16_t divider){
	LL_LPTIM_SetCompare(LPTIM2, divider / 2 - 1);
	LL_LPTIM_SetAutoReload(LPTIM2, divider - 1);
}

/* USER CODE BEGIN 1 */
//min value is 60 us, max value is 3.96s
void lptim_sleep_us(uint32_t u_sec){
	if (u_sec > 3967851){
		u_sec = 3967851;
	}
	u_sec=u_sec/61;
	LL_LPTIM_Enable(LPTIM1);
	LL_LPTIM_SetCompare(LPTIM1, u_sec);
	LL_LPTIM_SetAutoReload(LPTIM1, u_sec+1);
	__disable_irq();
	LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
	__WFI();
	__enable_irq();
}

void lptim_set_wakeup_irq_us(uint32_t u_sec){
	if (u_sec > 3967851){
		u_sec = 3967851;
	}
	u_sec=u_sec/61;
	LL_LPTIM_Enable(LPTIM1);
	LL_LPTIM_SetCompare(LPTIM1, u_sec);
	LL_LPTIM_SetAutoReload(LPTIM1, u_sec+1);
	__disable_irq();
	LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_ONESHOT);
	lptim_dbg_flg=1;
}

void lptim_disable(LPTIM_TypeDef *LPTIMx){
	LL_LPTIM_Disable(LPTIMx);
	LL_LPTIM_ClearFLAG_CMPM(LPTIMx);
	LL_LPTIM_ClearFLAG_ARRM(LPTIMx);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
