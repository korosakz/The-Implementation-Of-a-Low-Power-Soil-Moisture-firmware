/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* RTC init function */
void MX_RTC_Init(void)
{
	LL_EXTI_InitTypeDef EXTI_InitStruct;

	/* Peripheral clock enable */
	if(!LL_RCC_IsEnabledRTC()){
		LL_RCC_EnableRTC();
	}

	// RTC interrupt Init
	NVIC_SetPriority(RTC_WKUP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
	NVIC_EnableIRQ(RTC_WKUP_IRQn);

	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_20;
	EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

}

void rtc_set_time(const uint8_t* time_array, uint32_t time){
	LL_RTC_InitTypeDef RTC_InitStruct;
	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_DateTypeDef RTC_DateStruct;

	if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != time){
		/**Initialize RTC and set the Time and Date
		*/
		RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
		RTC_InitStruct.AsynchPrescaler = 127;
		RTC_InitStruct.SynchPrescaler = 255;
		LL_RTC_Init(RTC, &RTC_InitStruct);

		RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_THURSDAY;
		RTC_DateStruct.Year = time_array[6];
		RTC_DateStruct.Month = time_array[5];
		RTC_DateStruct.Day = time_array[4];
		LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_DateStruct);

		RTC_TimeStruct.Hours = time_array[3];
		RTC_TimeStruct.Minutes = time_array[2];
		RTC_TimeStruct.Seconds = time_array[1]*4;
		LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);

		LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0, time);
	}
}

/* USER CODE BEGIN 1 */
void set_rtc_wakeup(uint32_t time_in_s){
	//if(!LL_RTC_WAKEUP_IsEnabled(RTC) && LL_RTC_IsActiveFlag_WUTW(RTC)){
		LL_RTC_DisableWriteProtection(RTC);
		LL_RTC_DisableIT_WUT(RTC);

		LL_RTC_ClearFlag_WUT(RTC);
		LL_RTC_WAKEUP_Disable(RTC);
		LL_RTC_ClearFlag_WUT(RTC);

		while(!LL_RTC_IsActiveFlag_WUTW(RTC));

		LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);  //this sets timer clk to 1Hz
		LL_RTC_WAKEUP_SetAutoReload(RTC, time_in_s);

		LL_RTC_EnableIT_WUT(RTC);

		LL_RTC_WAKEUP_Enable(RTC);
		LL_RTC_EnableWriteProtection(RTC);
	//}
}

void disable_rtc_wakeup(){
	LL_RTC_DisableWriteProtection(RTC);

	LL_RTC_DisableIT_WUT(RTC);

	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	LL_RTC_ClearFlag_WUT(RTC);
}

uint32_t get_time_diff_in_minutes(const uint8_t* time_table){
	uint32_t time_diff = 0;
	uint32_t days_start, days_end;
	uint32_t current_hour, current_minute;
	days_start = year_month_to_days(time_table[4], time_table[5], time_table[6]);
	days_end = year_month_to_days(	__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
									__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
									__LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC))
									);

	time_diff =  days_end - days_start;
	current_hour = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
	current_minute = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));
	time_diff = (time_diff * 24 ) + current_hour - time_table[3];
	time_diff = (time_diff * 60 ) + current_minute - time_table[2];
	return time_diff & 0x7FFFFF; // use only first 23 bits
}

uint32_t year_month_to_days(uint8_t d,uint8_t m,uint8_t y)
{ uint32_t a,r[13];
  r[1] = 0; r[2] = 31; r[3] = 59;
  r[4] = 90; r[5] = 120; r[6] = 151;
  r[7] = 181; r[8] = 212; r[9] = 243;
  r[10]= 273; r[11]= 304; r[12]= 334;
  a=r[m]+d;
  if(((((y%400)==0)||((y%100)!=0))
      &&((y%4)==0))&&(m>2)) a+=1;
  return(a);
}

void decode_time(uint8_t* const time_table_ret, uint32_t time){
	uint32_t seconds, minutes, hours, day, month, year, n_lock;  //n_lock is for RTC on MCU. If it is high RTC on MCU will be set to given time

	n_lock = (time >> 31) & 0x01;
	seconds = (time >> 27) & 0x0F;
	minutes = (time >> 21) & 0x3F;
	hours = (time >> 16) & 0x1F;
	day = (time >> 11) & 0x1F;
	month = (time >> 7) & 0x0F;
	year = (time >> 0) & 0x7F;

	time_table_ret[0]=n_lock;
	time_table_ret[1]=seconds;
	time_table_ret[2]=minutes;
	time_table_ret[3]=hours;
	time_table_ret[4]=day;
	time_table_ret[5]=month;
	time_table_ret[6]=year;
}

uint32_t encode_time(const uint8_t* time_table){
	uint32_t time_var = 0;
	time_var = (time_var << 0 ) | (time_table[0] & 0x01);  // n_lock
	time_var = (time_var << 4 ) | (time_table[1] & 0x0F);  // seconds
	time_var = (time_var << 6 ) | (time_table[2] & 0x3F);  // minutes
	time_var = (time_var << 5 ) | (time_table[3] & 0x1F);  // hours
	time_var = (time_var << 5 ) | (time_table[4] & 0x1F);  // days
	time_var = (time_var << 4 ) | (time_table[5] & 0x0F);  // months
	time_var = (time_var << 7 ) | (time_table[6] & 0x7F);  // years
	return time_var;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
