
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *      without specific prior written permission.4
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
uint8_t wkup_irq_flg, configuration_changed;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	int16_t bps_temp=0, bps_hum=0, sl900_temp=0, sl900_light=0, moisture=0;
	uint16_t sampling_interval=0, free_mem_start, result_write_header;
	uint32_t time, time_diff;
	uint64_t compressed_data;
	uint8_t decoded_time[7], data_buffer[20], memory_full=True, time_ok = False;
	//uint16_t i;
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	LL_PWR_EnableLowPowerRunMode();        //enable this later
	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN1);
	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_RTC_Init();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_LPTIM1_Init();  //wakeup timer
	/* USER CODE BEGIN 2 */

	if(LL_PWR_IsActiveFlag_WU1()){
		LL_PWR_ClearFlag_WU1();

		lptim_sleep_us(5000);
		__disable_irq();
		if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) && !wkup_irq_flg){
			LL_LPM_EnableDeepSleep();
			LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
			__WFI();
		}
		__enable_irq();
//		//dbg
//		for (i=1; i<20; i++){
//			LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			lptim_sleep_us(800000);
//		}

		MX_SPI1_Init();
		LL_SPI_Enable(SPI1);
		sl_transfer_response_irq_sequence("\x40\x66", 2, 5, 0);
		time = convert_uint8_array_to_uint32((uint8_t*)spi_rx_buff);
		decode_time(decoded_time, time);
		if (decoded_time[0]==1){
			configuration_changed=1;
		}
	}

	if (!wkup_irq_flg || configuration_changed){

		//dbg
		LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
		lptim_sleep_us(200000);
		LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);

		//this section reads data from BPS230
		LL_I2C_Enable(I2C1);

		//CE for BPS230 may need some time to enable after setting pin
		LL_GPIO_SetOutputPin(CE1_GPIO_Port, CE1_Pin);

		LL_LPM_EnableSleep();
		lptim_sleep_us(200); //wait 200us

		i2c_tx_dma_blocking("\x01\x3D", 0xFF, 2, True); 	// make sure to check man/auto mode on BPS
		while(!LL_I2C_IsActiveFlag_STOP(I2C1)); //wait until stop is asserted, maybe set stop interrupt

		if (i2c_communication_successful){
			LL_LPM_EnableDeepSleep();
			LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
			lptim_sleep_us(110000); //wait 110ms for sensor to finish measuring
			LL_LPM_EnableSleep();

			i2c_tx_dma_blocking("\x03", 0xFF, 1, False);
			i2c_rx_dma_blocking(0xFF, 6);
		}
		LL_GPIO_ResetOutputPin(CE1_GPIO_Port, CE1_Pin);
		i2c_deinit();
		if (i2c_communication_successful){
			bps_temp = bps_convert_temp(3);
			bps_hum = bps_convert_hum(1);
		}

		// this section performs adc measurement
		MX_LPTIM2_Init(); //this enables pwm output

		MX_ADC1_Init();
		lptim_sleep_us(30000);  //wait for sensor to stabilise

		LL_ADC_Enable(ADC1);
		__WFI();

		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_REG_IsConversionOngoing(ADC1)){
			__WFI();
		}
		moisture = convert_adc_result(0, 1);

		if (moisture >= 590 && moisture <= 2412){
			moisture = (1 << 12) | moisture;
		}
		else if (moisture > 2412){
			lptim_set_divider(4);
			lptim_sleep_us(60000);
			LL_ADC_REG_StartConversion(ADC1);
			while(LL_ADC_REG_IsConversionOngoing(ADC1)){
				__WFI();
			}
			moisture = convert_adc_result(2, 0);
		}
		else{
			lptim_set_divider(18);
			lptim_sleep_us(30000);
			LL_ADC_REG_StartConversion(ADC1);
			while(LL_ADC_REG_IsConversionOngoing(ADC1)){
				__WFI();
			}
			moisture = convert_adc_result(2, 2);
			moisture = (2 << 12) | moisture;
		}

		lptim_disable(LPTIM2);

		if (!LL_SPI_IsEnabled(SPI1)){
			MX_SPI1_Init();
			LL_SPI_Enable(SPI1);
		}

		sl_transfer_response_irq_sequence("\x40\x64", 2, 11, 4);
		lptim_sleep_us(SL900_SLEEP_US);
		free_mem_start = convert_uint8_array_to_uint16((uint8_t*)spi_rx_buff+4);
		time = convert_uint8_array_to_uint32((uint8_t*)spi_rx_buff+2+4);
		sampling_interval = convert_uint8_array_to_uint16((uint8_t*)spi_rx_buff+6+4);
		decode_time(decoded_time, time);

		if (sl900_exti_flag){
			if (decoded_time[1]<20 && decoded_time[2]<60 && decoded_time[3]<24 && decoded_time[4]<31 && decoded_time[5]<12 && decoded_time[6]<99){
				time_ok = True;
				if (decoded_time[0]==1){
					configuration_changed=1;

					decoded_time[0]=0; //if this flag is set RTC will be reset and then flag will be removed
					time = encode_time(decoded_time);

					memcpy(data_buffer, "\x08\x66", 2);
					memcpy(data_buffer+2, &time, sizeof(time));
					sl_transfer_wait_sequence((char*)data_buffer, 6, 10000);  //un-comment this in final version
					result_write_header = 0x006E + free_mem_start;
					data_buffer[0]=result_write_header >> 8;
					data_buffer[1]=result_write_header;
					memcpy(data_buffer+2, "\x00\x00", 2);
					sl_transfer_wait_sequence((char*)data_buffer, 3, 10000);
				}
			rtc_set_time(decoded_time, time);  // if pwr is lost RTC will be reset
			}
			if (sampling_interval>0){
				sl_transfer_response_irq_sequence(SL900_GET_TEMP_CMD, 1, 2, 0);  //cmd meas temp
				lptim_sleep_us(SL900_SLEEP_US);
				sl900_temp=sl_convert_temp(0);

				sl_transfer_response_irq_sequence(SL900_GET_EXT_SEN2_CMD, 1, 2, 2);  //cmd meas light
				lptim_sleep_us(SL900_SLEEP_US);
				sl900_light=sl_convert_light(2);

				time_diff = get_time_diff_in_minutes(decoded_time);  //24bits
				time_diff = (time_diff << 1) | 0x1; //set 1st bit to 1 to signal write slot full
				compressed_data = compress_measurement_data(bps_hum, (bps_temp+sl900_temp)/2, sl900_light, moisture);  //48bits
				memory_full = save_measurement_data_to_eeprom(compressed_data, time_diff, free_mem_start);
			}
		}
	}

//	sl_transfer_response_irq_sequence("\x40\x6E", 2, 38, 0);
	while (1)
	{
		if (sampling_interval>0 && memory_full == False && time_ok == True){
//			LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			LL_LPM_EnableDeepSleep();
//			LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
//			lptim_sleep_us(1000000);

			set_rtc_wakeup(sampling_interval);
		}
		else if(wkup_irq_flg){

		}
		else{
			LL_RTC_DisableWriteProtection(RTC);
			LL_RTC_DisableIT_WUT(RTC);

			LL_RTC_ClearFlag_WUT(RTC);
			LL_RTC_WAKEUP_Disable(RTC);
			LL_RTC_ClearFlag_WUT(RTC);

		}
		LL_LPM_EnableDeepSleep();
		LL_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
		__WFI();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

static void LL_Init(void)
{
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

	if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
	{
	Error_Handler();
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

	LL_PWR_EnableBkUpAccess();  // this must be set to enable rtc usage and more
	if (!LL_RCC_LSE_IsReady() || !LL_RCC_IsEnabledRTC()){

		LL_RCC_ForceBackupDomainReset();
		LL_RCC_ReleaseBackupDomainReset();

		LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
		LL_RCC_LSE_Enable();

		/* Wait till LSE is ready */
		while(LL_RCC_LSE_IsReady() != 1);
		LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSE);

		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
		//LL_RCC_EnableRTC();
	}

	LL_RCC_MSI_Enable();

	/* Wait till MSI is ready */
	while(LL_RCC_MSI_IsReady() != 1)
	{

	}
	LL_RCC_MSI_EnablePLLMode();
	LL_RCC_MSI_EnableRangeSelection();
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5);
	LL_RCC_MSI_SetCalibTrimming(0);

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
	{

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_Init1msTick(2000000);
	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

	LL_SetSystemCoreClock(2000000);
	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSE);
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM2_CLKSOURCE_PCLK1);
	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);

	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
