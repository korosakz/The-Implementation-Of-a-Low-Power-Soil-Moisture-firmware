/**
  ******************************************************************************
  * File Name          : SPI.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_ll_spi.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "lptim.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define SPI_TX_BUFF_SIZE 20
#define SPI_RX_BUFF_SIZE 40
/* USER CODE END Private defines */
extern char spi_tx_buff[SPI_TX_BUFF_SIZE];
extern char spi_rx_buff[SPI_RX_BUFF_SIZE];
/* USER CODE BEGIN Prototypes */
 extern uint8_t sl900_exti_flag;

 extern void _Error_Handler(char *, int);

 void MX_SPI1_Init(void);


 void spi_tx_rx_dma_enable(char *send_buf, uint8_t n_bytes);
 void spi_tx_rx_dma_enable_blocking(char *send_buf, uint8_t n_bytes);

 void spi_tx_dma_enable(char *send_buf, uint8_t n_bytes);
 void spi_tx_dma_enable_blocking(char *send_buf, uint8_t n_bytes);

 void spi_rx_dma_n_bytes(uint8_t n_bytes, uint8_t buffer_offset);
 void spi_rx_dma_n_bytes_blocking(uint8_t n_bytes, uint8_t buffer_offset);

 int16_t sl_convert_temp(uint8_t buffer_offset);
 int16_t sl_convert_light(uint8_t buffer_offset);

 void sl_transfer_response_irq_sequence(char *send_buf, uint8_t n_bytes_tx, uint8_t n_bytes_rx, uint8_t buffer_offset);
 void sl_transfer_wait_sequence(char *send_buf, uint8_t n_bytes_tx, uint32_t sleep_time);
 uint64_t compress_measurement_data(int16_t hum, int16_t temp, int16_t light, int16_t moist);
 uint8_t save_measurement_data_to_eeprom(uint64_t compressed_data, uint32_t time_diff, uint16_t free_mem_start);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
