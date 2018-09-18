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

#define TXBUFFERSIZE_U1   		0x20
//uint8_t TxBuffer_U1[TXBUFFERSIZE_U1] = {0}; //transmitting byte per byte

#define	TASK1_BIT	(1<<0)
#define	TASK2_BIT	(1<<1)
#define	TASK3_BIT	(1<<2)
#define	TASK4_BIT	(1<<3)
#define	TASK5_BIT	(1<<4)
#define	TASK6_BIT	(1<<5)
#define	TASK7_BIT	(1<<6)
#define	TASK8_BIT	(1<<7)
#define	TASK9_BIT	(1<<8)
#define	TASK10_BIT	(1<<9)

#define	TASK_BIT_ALL	0x1FF

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
	uint8_t 	CHARGE_STATUS;
	uint8_t 	STATUS;
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_CHARGING_ACK;

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
	uint8_t		STATUS;	
	uint8_t		CRC_HIGH;
	uint8_t		CRC_LOW;
}UART1_FIRMWARE_ACK;

typedef enum
{
  CMD_CHARGING = 0x11,	
	CMD_UART_TRANSFER = 0x12,
	CMD_FIRMWARE = 0x13, 
	CMD_CHARGING_ACK = 0x91,	
	CMD_UART_TRANSFER_ACK = 0x92,
	CMD_FIRMWARE_ACK = 0x93 	
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
	STATUS_PHONE2PACK,
	STATUS_SLEEP
}PACK_STATUS;


/* Exported functions ------------------------------------------------------- */
void SystemClock_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void ADC_Data_Handle(void);
void set_pack_status(PACK_INFO *rptr);

static void UART2_task1(void const *argument);
static void ADC_task2(void const *argument);
static void CHG_task3(void const *argument);
static void SMB_task4(void const *argument)	;
static void LED_task5(void const *argument)	;
static void UART1_RX_task6(void const *argument);
static void UART1_TX_task7(void const *argument);
static void UART1_RX_HANDLE_task8(void const *argument);
static void ATTACH_task9(void const *argument);
static void WDG_task10(void const *argument);

PACK_STATUS get_pack_status(PACK_INFO *ptr);
static void SYSCLKConfig_STOP(void);
void DecodeReception(uint16_t *d1,uint16_t *d2);
void pbfw(void);
void pbsoc(void);
void pbphsoc(uint8_t soc);
void pbusb(uint16_t usb_vol);
void pbwr(uint16_t addr, uint16_t data);
void pbrd(uint16_t addr);
void pbhelp(void);
void pbadc(void);
void pbwrcw(uint16_t addr, uint16_t data);
void pbrdcw(uint16_t addr);
void led_of_soc(uint8_t soc, uint8_t key);
void pbpmuxena(uint8_t io);
void pbpmuxenb(uint8_t io);
void pbchgen(uint8_t io);
void pbboosten(uint8_t en);
void pbboost9ven(uint8_t en);
void set_pack_path(uint8_t path);
void usb_charging(USB_TYPE usb_type);
void delayms(uint32_t);
void usb_charging_task(USB_TYPE usb_type);
void uart1_rx_no_ack_handle(uint8_t *rxdata, PHONE_INFO *info);
uint16_t crc16(uint8_t *data, uint8_t data_len);
void uart1_rx_ack_handle(uint8_t *uart_rxdata, uint8_t *queue_num, UART1_TX_QUEUE *tx_ack_queue, PHONE_INFO *info);
void invert_uint8(uint8_t *dbuff, uint8_t *srcbuff);
void invert_uint16(uint16_t *dbuff, uint16_t *srcbuff);
void uart1_tx_charging_data_handle(USB_TYPE *usb_type, PACK_INFO *pk_info,UART1_TX_QUEUE *tx_queue);
void led_flash(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
