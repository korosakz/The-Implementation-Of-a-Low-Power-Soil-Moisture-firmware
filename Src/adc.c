/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

#include "gpio.h"

#define ADC_CORRECTION_V -0.006
/* USER CODE BEGIN 0 */
uint16_t adc_data[ADC_BUFFER_LEN];
/* USER CODE END 0 */

/* ADC1 init function */
void MX_ADC1_Init(void)
{

	LL_ADC_InitTypeDef ADC_InitStruct;
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;
	/* Peripheral clock enable */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	/**ADC1 GPIO Configuration
	PA3   ------> ADC1_IN8
	PA1   ------> ADC1_IN6
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	LL_LPM_EnableSleep();
	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);
	lptim_sleep_us(400);

	/* ADC1 interrupt Init */
	NVIC_SetPriority(ADC1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
	NVIC_EnableIRQ(ADC1_IRQn);

	/**Common config
	*/
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);

	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
//	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
	LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_64, LL_ADC_OVS_SHIFT_RIGHT_4);

	ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

	//LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);

	LL_ADC_EnableIT_ADRDY(ADC1);
	LL_ADC_EnableIT_EOC(ADC1);
	LL_ADC_EnableIT_EOS(ADC1);

	/**Configure Regular Channel
	*/
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_VREFINT);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_24CYCLES_5); // minimum sampling time is 4 us
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);
	//make sure to wait for callibration to finish
}

float convert_adc_result(uint8_t adc_data_offset, uint8_t range){
	float temp =  adc_data[adc_data_offset], vsup, vf, vf_vsup_correction;
	volatile uint16_t vref_cal = *(uint16_t*) (0x1FFF75AA);
	vsup = (float)(12 * vref_cal) / (adc_data[adc_data_offset+1]);
	vf_vsup_correction = 0.016 * vsup - 0.0528;
	temp = temp / 16383;
	vf = get_diode_voltage(range, temp);
	temp = ((temp - 0.65) * vsup + ADC_CORRECTION_V + vf + vf_vsup_correction) / vsup; //move result closer to 0 by substracting 0.65 instead of 0.5
	if (temp < 0)
		temp = 0;
	 return round(temp*10000);
}

float get_diode_voltage(uint8_t range, float value){
	if (range == 0){
		if (value>0.75){
			return 0.3666 - 0.00199 * 100 * value;
		}
		else{
			return 0.1503 + 0.00076 * 100 * value;
		}
	}
	else if (range == 1){
		if (value>0.72){
			return 0.2956 - 0.00114 * 100 * value;
		}
		else{
			return 0.1092 + 0.00142 * 100 * value;
		}
	}
	else{
		if (value>0.70){
			return 0.2437 - 0.00048 * 100 * value;
		}
		else{
			return 0.1534 + 0.00077 * 100 * value;
		}
	}

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
