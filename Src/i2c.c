/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
uint8_t i2c_communication_successful = 0;
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{
	LL_I2C_InitTypeDef I2C_InitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/**I2C1 GPIO Configuration
	PA9   ------> I2C1_SCL
	PA10   ------> I2C1_SDA
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	/* I2C1 DMA Init */

	/* I2C1_TX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_3);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)&I2C1->TXDR);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
	LL_I2C_EnableDMAReq_TX(I2C1);

	/* I2C1_RX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_7, LL_DMA_REQUEST_3);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)&I2C1->RXDR);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
	LL_I2C_EnableDMAReq_RX(I2C1);

	/**I2C Initialization*/
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	I2C_InitStruct.Timing = 0x00200D13;
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C1, &I2C_InitStruct);

	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_DisableOwnAddress2(I2C1);
	LL_I2C_DisableGeneralCall(I2C1);
	LL_I2C_EnableClockStretching(I2C1);
	//LL_I2C_DisableClockStretching(I2C1);  //BPS does not seem to support clock stretching

}

/* USER CODE BEGIN 1 */
char i2c_tx_buff[I2C_TX_BUFF_SIZE];
char i2c_rx_buff[I2C_RX_BUFF_SIZE];
uint16_t tx_buf_idx;
uint16_t tx_buf_send_len;

void i2c_deinit(){
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);
	LL_I2C_Disable(I2C1);
	LL_I2C_DeInit(I2C1);
}

// this function send slave address and expects slave to respond. Address of register to read must be send prior with tx function
void i2c_rx_dma(uint8_t slave_addr, uint8_t n_bytes){
	// RX
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)i2c_rx_buff);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, n_bytes);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

	LL_I2C_SetSlaveAddr(I2C1, slave_addr);
	LL_I2C_SetTransferSize(I2C1, n_bytes);
	LL_I2C_EnableAutoEndMode(I2C1);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ);
	LL_I2C_GenerateStartCondition(I2C1);
}

void i2c_rx_dma_blocking(uint8_t slave_addr, uint8_t n_bytes){
	__disable_irq();
	i2c_rx_dma(slave_addr, n_bytes);
	__WFI();
	//transfer completed interrupt must trigger here.
	__enable_irq();
}

// this function  sends data over i2c. It should also be used as a part of rx process to send the register address before reading out
void i2c_tx_dma(char *send_buf, uint8_t slave_addr, uint8_t n_bytes, uint8_t autoend){
	// TX
	memcpy (i2c_tx_buff, send_buf, n_bytes );
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)i2c_tx_buff);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, n_bytes);
	if (n_bytes)
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

	LL_I2C_SetSlaveAddr(I2C1, slave_addr);
	LL_I2C_SetTransferSize(I2C1, n_bytes);
	if (autoend)
		LL_I2C_EnableAutoEndMode(I2C1);
	else
		LL_I2C_DisableAutoEndMode(I2C1);
	LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_WRITE);
	LL_I2C_GenerateStartCondition(I2C1);
}

void i2c_tx_dma_blocking(char *send_buf, uint8_t slave_addr, uint8_t n_bytes, uint8_t autoend){
	lptim_set_wakeup_irq_us(4000);
	__disable_irq();
	i2c_tx_dma(send_buf, slave_addr, n_bytes, autoend);
	__WFI();
	//transfer completed interrupt must trigger here.
	__enable_irq();
	lptim_disable(LPTIM1);
}

int16_t bps_convert_temp(uint8_t buffer_offset){
	uint8_t byte1, byte2;
	int16_t output;
	byte1 = i2c_rx_buff[buffer_offset];
	byte2 = i2c_rx_buff[buffer_offset+1];
	output = (byte2 << 8) + byte1;
	output = (output - 774); // output is in format xx,x
	return output;
}

int16_t bps_convert_hum(uint8_t buffer_offset){
	uint8_t byte1, byte2;
	int16_t output;
	byte1 = i2c_rx_buff[buffer_offset];
	byte2 = i2c_rx_buff[buffer_offset+1];
	output = (byte2 << 8) + byte1;  //output is 0-1023 and coresponds to 0-100%
	return output;
}
// LL_I2C_GetTimingPrescaler(I2C_TypeDef *I2Cx)

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
