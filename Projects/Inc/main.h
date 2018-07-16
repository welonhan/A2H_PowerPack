/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
//#include "p9221.h"
#include "smb1381.h"
#include "cw2015.h"
#include "power_pack.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


#define __PACK_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __PACK_BSP_VERSION_SUB   	(0x00) /*!< [23:16] sub1 version */

//#define __PACK_BSP_VERSION        ((__PACK_BSP_VERSION_MAIN << 8)\
                                                 |(__PACK_BSP_VERSION_SUB))
                                            
#define TXBUFFERSIZE_U1   		0x20
//uint8_t TxBuffer_U1[TXBUFFERSIZE_U1] = {0}; //transmitting byte per byte

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t 	ATTACH;
	uint8_t 	USB;
	uint8_t 	DCIN;
	uint8_t		PHONE_USB;
	uint8_t 	PHONE_SOC;
	uint8_t 	PACK_SOC;
	uint16_t	PHONE_USB_VOLTAGE;
}PACK_INFO;

typedef struct
{
	uint8_t		SOF_HIGH;
	uint8_t		SOF_LOW;
	uint8_t		LENGTH;
	uint8_t		CMD_TYPE;
	uint8_t		CMD;
	uint8_t		VOLTAGE_HIGH;
	uint8_t		VOLTAGE_LOW;
	uint8_t		CURRENT_HIGH;
	uint8_t		CURRENT_LOW;
	uint8_t		SOC;
	uint8_t		ACC;
	uint8_t 	STATUS;
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_CHARGING;

typedef struct
{
	uint8_t		SOF_HIGH;
	uint8_t		SOF_LOW;
	uint8_t		LENGTH;
	uint8_t		CMD_TYPE;
	uint8_t		CMD;
	uint8_t		NUM;
	uint8_t		SIZE;
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_TRANSFER_INITIAL;

typedef struct
{
	uint8_t		SOF_HIGH;
	uint8_t		SOF_LOW;
	uint8_t		LENGTH;
	uint8_t		CMD_TYPE;
	uint8_t		DATA[16];
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_TRANSFER_DATA;

typedef struct
{
	uint8_t		SOF_HIGH;
	uint8_t		SOF_LOW;
	uint8_t		LENGTH;
	uint8_t		CMD_TYPE;
	uint8_t 	CMD;
	uint8_t		MVERSION;
	uint8_t		SVERSION;	
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_FIRMWARE;

typedef enum
{
  CMD_CHARGING = 0x11,
	CMD_UART_TRANSFER = 0x12,
	CMD_FIRMWARE = 0x13  
} CMD_TYPE;

typedef enum
{
  ACC_NULL = 0,
  ACC_CAMERA	
} ACC_TypeDef;

typedef enum
{
  PHONE_NO_ACK = 0,
  PHONE_ACK	
} PH_ACK;

typedef enum
{
  PACK_NO_ACK = 0,
  PACK_ACK	
} PK_ACK;

typedef struct
{
	PK_ACK		ACK_PACK;
	uint8_t 	TX_BUF[TXBUFFERSIZE_U1];
}UART1_TX_QUEUE;

typedef struct
{
	PH_ACK 		ACK_PHONE;
	CMD_TYPE	CMD_T;	
}UART1_RX_QUEUE;

typedef struct
{
	uint16_t	EXTPWR_VOLTAGE;
	uint8_t 	PHONE_SOC;
}PHONE_INFO;

typedef enum
{
	STATUS_STANDBY=0,
	STATUS_USB2PACK,
	STATUS_USB2BOTH,
	STATUS_USB2PHONE,
	STATUS_BOOST2PHONE,
	STATUS_PHONE2PACK
}PACK_STATUS;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
