/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#include "spi.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
uint8_t sl900_exti_flag=False;
/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/**SPI1 GPIO Configuration
	PA5   ------> SPI1_SCK
	PA7   ------> SPI1_MOSI
	PA11   ------> SPI1_MISO
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7|LL_GPIO_PIN_11;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* SPI1 DMA Init */
	/* SPI1_RX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_1);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&SPI1->DR);

	/* SPI1_TX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

	LL_SPI_EnableDMAReq_TX(SPI1);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&SPI1->DR);

	/* SPI Init */
	SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI1, &SPI_InitStruct);

	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
	LL_SPI_EnableNSSPulseMgt(SPI1);
}

/* USER CODE BEGIN 1 */
char spi_tx_buff[SPI_TX_BUFF_SIZE];
char spi_rx_buff[SPI_RX_BUFF_SIZE];
uint16_t tx_buf_idx;
uint16_t tx_buf_send_len;

void spi_tx_rx_dma_enable(char *send_buf, uint8_t n_bytes){
	// RX
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&SPI1->DR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)spi_rx_buff);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, n_bytes);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	// TX
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&SPI1->DR);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
	memcpy (spi_tx_buff, send_buf, n_bytes );
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)spi_tx_buff);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, n_bytes);
	if (n_bytes)
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

// Sleep should be enabled when calling this. Deep sleep must not be enabled, or application will hang.
void spi_tx_rx_dma_enable_blocking(char *send_buf, uint8_t n_bytes){
	__disable_irq();
	spi_tx_rx_dma_enable(send_buf, n_bytes);
	__WFI();
	//transfer completed interrupt must trigger here.
	__enable_irq();
}

void spi_tx_dma_enable(char *send_buf, uint8_t n_bytes)  //this function will not send zeros
{
	// TX
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_HALF_DUPLEX_TX);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
	memcpy (spi_tx_buff, send_buf, n_bytes );
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)spi_tx_buff);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, n_bytes);
	if (n_bytes)
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

// Sleep should be enabled when calling this. Deep sleep must not be enabled, or application will hang.
void spi_tx_dma_enable_blocking(char *send_buf, uint8_t n_bytes){
	__disable_irq();
	spi_tx_dma_enable(send_buf, n_bytes);
	__WFI();
	//transfer completed interrupt must trigger here.
	__enable_irq();
}

void spi_rx_dma_n_bytes(uint8_t n_bytes, uint8_t buffer_offset)
{
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)spi_rx_buff+buffer_offset);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, n_bytes);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)spi_tx_buff);
	spi_tx_buff[0]=0;
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, n_bytes); //it seems this must be one more than rx, or rx wont finish
	if (n_bytes){
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	}
}

// Sleep should be enabled when calling this. Deep sleep must not be enabled, or application will hang.
void spi_rx_dma_n_bytes_blocking(uint8_t n_bytes, uint8_t buffer_offset){
	__disable_irq();
	spi_rx_dma_n_bytes(n_bytes, buffer_offset);
	__WFI();
	//transfer completed interrupt must trigger here.
	__enable_irq();
}

int16_t sl_convert_temp(uint8_t buffer_offset){
	uint8_t byte1, byte2;
	int16_t output;
	byte2 = spi_rx_buff[buffer_offset];
	byte1 = spi_rx_buff[buffer_offset+1];
	output = (byte2 << 8) + byte1;
	output = (output*0.855 - 293); // output is in format xx,x
	return output;
}

int16_t sl_convert_light(uint8_t buffer_offset){
	uint8_t byte1, byte2;
	uint16_t range;
	uint16_t output;
	range = (spi_rx_buff[buffer_offset] & 0x7C) >> 2;
	switch (range){
		case 0x01:
			range = 3875;
			break;
		case 0x02:
			range = 1875;
			break;
		case 0x04:
			range = 875;
			break;
		case 0x08:
			range = 400;
			break;
		case 0x10:
			range = 185;
			break;
	}
	byte2 = spi_rx_buff[buffer_offset] & 0x03;
	byte1 = spi_rx_buff[buffer_offset+1];
	output = (byte2 << 8) + byte1;
	output = ((uint32_t)output*3027 + 1750000) / (10*range); // output is in format xxxx nA
	return output;
}

void sl_transfer_response_irq_sequence(char *send_buf, uint8_t n_bytes_tx, uint8_t n_bytes_rx, uint8_t buffer_offset){
	LL_GPIO_SetOutputPin(CS_GPIO_Port, CS_Pin);
	LL_LPM_EnableSleep();
	gpio_set_exti_on_miso();
	sl900_exti_flag=False;

	lptim_sleep_us(100);
	lptim_set_wakeup_irq_us(20000);
	spi_tx_dma_enable_blocking(send_buf, n_bytes_tx);
	//dma transfer finished irq will trigger here
	__disable_irq();
	if (sl900_exti_flag == False){
		LL_LPM_EnableDeepSleep();
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
		__WFI();
	}
	__enable_irq();
	//sensor finished interrupt will trigger here

	if (sl900_exti_flag == True){
		LL_LPM_EnableSleep();
		spi_rx_dma_n_bytes_blocking(n_bytes_rx, buffer_offset);
	}
	LL_GPIO_ResetOutputPin(CS_GPIO_Port, CS_Pin);
}

void sl_transfer_wait_sequence(char *send_buf, uint8_t n_bytes_tx, uint32_t sleep_time){
	LL_GPIO_SetOutputPin(CS_GPIO_Port, CS_Pin);
	LL_LPM_EnableSleep();
	spi_tx_dma_enable_blocking(send_buf, n_bytes_tx);
	lptim_sleep_us(SL900_SLEEP_US);
	LL_GPIO_ResetOutputPin(CS_GPIO_Port, CS_Pin);
	LL_LPM_EnableDeepSleep();
	lptim_sleep_us(sleep_time); //wait 20ms
}

uint64_t compress_measurement_data(int16_t hum, int16_t temp, int16_t light, int16_t moist){
	uint64_t compressed_measurement_data;
	compressed_measurement_data = (hum & 0x03FF);
	compressed_measurement_data = (compressed_measurement_data << 12) | (temp & 0x0FFF);
	compressed_measurement_data = (compressed_measurement_data << 12) | (light & 0x0FFF);
	compressed_measurement_data = (compressed_measurement_data << 14) | (moist & 0x3FFF);
	return compressed_measurement_data;
}

uint8_t save_measurement_data_to_eeprom(uint64_t compressed_data, uint32_t time_diff, uint16_t free_mem_start){
	uint8_t break_out=False, block_remainder, data_buffer[16], ret_value = False;
	uint16_t result_write_offset, sub_offset, result_write_header, read_header;

	memcpy(data_buffer+2, &time_diff, 3);  //two usless bytes from previous write will be overwritten
	memcpy(data_buffer+5, &compressed_data, 8);
	data_buffer[11]=0;
	break_out=False;
	for ( result_write_offset = free_mem_start; result_write_offset < 1035; result_write_offset += 45){
		read_header = 0x406E + result_write_offset;
		data_buffer[0]=read_header >> 8;
		data_buffer[1]=read_header;
		sl_transfer_response_irq_sequence((char*)data_buffer, 2, 39, 0);
		for (sub_offset=0; sub_offset<37; sub_offset += 9){
			if ((spi_rx_buff[sub_offset] << 7 & 0xFF) != 128){
				result_write_header = 0x086E + result_write_offset + sub_offset;
				block_remainder=16 - (0x6E + result_write_offset + sub_offset) % 16; //space in current block
				if (result_write_header > 0xC76){
					ret_value = True;
				}
				if (block_remainder==1){
					result_write_header -= 0x0800;  // comand for non page write, as only one byte will be written
				}
				data_buffer[0]=result_write_header >> 8;
				data_buffer[1]=result_write_header;
				sl_transfer_wait_sequence((char*)data_buffer, block_remainder+2, 10000);

				if (block_remainder<10){
					if (block_remainder == 9){
						result_write_header = 0x006E + result_write_offset + sub_offset + block_remainder;  //regualar write
					}
					else{
						result_write_header = 0x086E + result_write_offset + sub_offset + block_remainder;  //page write
					}
					data_buffer[block_remainder]=result_write_header >> 8;
					data_buffer[block_remainder+1]=result_write_header;
					sl_transfer_wait_sequence((char*)(data_buffer+block_remainder), 12-block_remainder, 10000);
				}

//				result_write_header = 0x406E + result_write_offset + sub_offset;
//				data_buffer[0]=result_write_header >> 8;
//				data_buffer[1]=result_write_header;
//				sl_transfer_response_irq_sequence((char*)data_buffer, 2, 14, 4);

				if ( result_write_offset + sub_offset > free_mem_start+100){
					memcpy(data_buffer, "\x08\x64", 2);
					free_mem_start = result_write_offset + sub_offset + 9;
					memcpy(data_buffer+2, &free_mem_start, 2);
					sl_transfer_wait_sequence((char*)data_buffer, 4, 10000);  //change free_mem_start addr
				}
				break_out=True;
				break;
			}
		}
		if (break_out){
			break;
		}
	}
	return ret_value;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
