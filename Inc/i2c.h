/**
  ******************************************************************************
  * File Name          : I2C.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"
#include "main.h"
#include "string.h"
#include "lptim.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define I2C_TX_BUFF_SIZE 10
#define I2C_RX_BUFF_SIZE 10
/* USER CODE END Private defines */
 extern uint8_t i2c_communication_successful;
/* USER CODE BEGIN Prototypes */

 extern void _Error_Handler(char *, int);

 void MX_I2C1_Init(void);
 void i2c_rx_dma(uint8_t slave_addr, uint8_t n_bytes);
 void i2c_rx_dma_blocking(uint8_t slave_addr, uint8_t n_bytes);

 void i2c_tx_dma(char *send_buf, uint8_t slave_addr, uint8_t n_bytes, uint8_t autoend);
 void i2c_tx_dma_blocking(char *send_buf, uint8_t slave_addr, uint8_t n_bytes, uint8_t autoend);

 int16_t bps_convert_temp(uint8_t buffer_offset);
 int16_t bps_convert_hum(uint8_t buffer_offset);
 void i2c_deinit();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
