/**
  ******************************************************************************
  * @file    power_pack.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-February-2016
  * @brief   This file contains definitions for:
  *          - LED available on STM32L4xx-Nucleo_32 Kit from STMicroelectronics
  *          - 7 segment display from Gravitech
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#ifndef __POWER_PACK_H
#define __POWER_PACK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

	 /**
  * @brief STM32L4XX NUCLEO BSP Driver version number V1.0.0
  */

 
 /*##################### GPIO MAP ###################################*/
	 
 /*##################### PA ###################################*/
 #define SMB_SUSP_PIN             	GPIO_PIN_0			//ADC1_IN5
 #define ACC_ID_ADC1_IN6_PIN        GPIO_PIN_1			
 #define BSP_UART2_TX_PIN           GPIO_PIN_2			
 #define BSP_UART2_RX_PIN           GPIO_PIN_3
 #define USB_IN_ADC1_IN9_PIN        GPIO_PIN_4			//ADC1_IN9
 #define SMB_RESET_PIN              GPIO_PIN_5
 #define EXT_PWR_ADC1_IN11_PIN      GPIO_PIN_6
 #define BOOST_ENABLE_PIN           GPIO_PIN_7
 #define BOOST_9V_EN_PIN            GPIO_PIN_8
 #define BSP_UART1_TX_PIN           GPIO_PIN_9
 #define BSP_UART1_RX_PIN           GPIO_PIN_10
 //#define                    			GPIO_PIN_11
 #define CHG_PATH_EN_PIN            GPIO_PIN_12
 //#define PHONE_ATTACHED_PIN       GPIO_PIN_13
 //#define LED1_PIN                 GPIO_PIN_14
 #define SMB_LPMODE_EN_PIN        	GPIO_PIN_15

 #define SMB_SUSP_PIN_GPIO_PORT              	GPIOA
 #define ACC_ID_ADC1_IN6_PIN_GPIO_PORT        GPIOA
 #define BSP_UART2_TX_PIN_GPIO_PORT           GPIOA 
 #define BSP_UART2_RX_PIN_GPIO_PORT           GPIOA
 #define USB_IN_ADC1_IN9_PIN_GPIO_PORT        GPIOA
 #define SMB_RESET_PIN_GPIO_PORT             	GPIOA
 #define EXT_PWR_ADC1_IN11_PIN_GPIO_PORT      GPIOA 
 #define BOOST_ENABLE_PIN_GPIO_PORT           GPIOA
 #define BOOST_9V_EN_PIN_GPIO_PORT         		GPIOA 
 #define BSP_UART1_TX_PIN_GPIO_PORT           GPIOA
 #define BSP_UART1_RX_PIN_GPIO_PORT           GPIOA
 #define CHG_PATH_EN_PIN_GPIO_PORT          	GPIOA
 #define SMB_LPMODE_EN_PIN_GPIO_PORT          GPIOA


 /*##################### PB ###################################*/
 #define SMB_CC_STS_PIN                     GPIO_PIN_0
 #define SMB_STAT_PIN                      	GPIO_PIN_1
 #define SMB_SYS_PIN                        GPIO_PIN_2
 #define BOOST_OCP_INT_PIN                  GPIO_PIN_3
 #define PHONE_ATTACHED_PIN                 GPIO_PIN_4
 #define KEY_PIN                   					GPIO_PIN_5
 #define PMUX1_CTRL_PIN                	  	GPIO_PIN_6
 #define PMUX1_MODE_PIN               	  	GPIO_PIN_7
 #define PMUX1_CHA_EN_PIN                   GPIO_PIN_8
 #define PMUX1_CHB_EN_PIN                   GPIO_PIN_9
 #define BSP_I2C2_SCL_PIN                   GPIO_PIN_10
 #define BSP_I2C2_SDA_PIN                   GPIO_PIN_11
 #define FG_INT_PIN                       	GPIO_PIN_12
 #define LED1_PIN                	  				GPIO_PIN_13
 #define LED2_PIN                	  				GPIO_PIN_14
 #define LED3_PIN                           GPIO_PIN_15
 
 #define SMB_CC_STS_PIN_GPIO_PORT          	GPIOB
 #define SMB_STAT_PIN_GPIO_PORT          		GPIOB
 #define SMB_SYS_PIN_GPIO_PORT          		GPIOB
 #define BOOST_OCP_INT_PIN_GPIO_PORT        GPIOB
 #define PHONE_ATTACHED_PIN_GPIO_PORT    		GPIOB
 #define KEY_PIN_GPIO_PORT          				GPIOB
 #define PMUX1_CTRL_PIN_GPIO_PORT         	GPIOB
 #define PMUX1_MODE_PIN_GPIO_PORT         	GPIOB
 #define PMUX1_CHA_EN_PIN_GPIO_PORT         GPIOB
 #define PMUX1_CHB_EN_PIN_GPIO_PORT         GPIOB 
 #define BSP_I2C2_SCL_PIN_GPIO_PORT         GPIOB
 #define BSP_I2C2_SDA_PIN_GPIO_PORT         GPIOB
 #define FG_INT_PIN_PIN_GPIO_PORT           GPIOB
 #define LED1_PIN_GPIO_PORT           			GPIOB
 #define LED2_PIN_GPIO_PORT           			GPIOB
 #define LED3_PIN_GPIO_PORT           			GPIOB
 

 /*##################### PC ###################################*/
 //#define SMB_CC_STS_PIN                    GPIO_PIN_0
 //#define SMB_STAT_PIN                      GPIO_PIN_1
 //#define SMB_SYS_PIN                       GPIO_PIN_2
 //#define BOOST_OCP_INT_PIN                 GPIO_PIN_3
 #define BSP_UART3_TX_PIN                  	 GPIO_PIN_4
 #define BSP_UART3_RX_PIN                 	 GPIO_PIN_5
 #define GPIO1_F_PIN                	 		 	 GPIO_PIN_6
 #define GPIO2_F_PIN                	 		   GPIO_PIN_7
 #define CAM_RESET_F_PIN                     GPIO_PIN_8
 //#define PMUX1_CHB_EN_PIN                  GPIO_PIN_9
 //#define ACC_UART3_TX_PIN                  GPIO_PIN_10
 //#define                   GPIO_PIN_10
 //#define FG_INT_PIN                        GPIO_PIN_12
 #define LED4_PIN                            GPIO_PIN_13
 //#define WLC_INT_PIN                		   GPIO_PIN_14
 //#define WLC_EN_PIN                        GPIO_PIN_15
 
 #define BSP_UART3_TX_PIN_GPIO_PORT          GPIOC
 #define BSP_UART3_RX_PIN_GPIO_PORT          GPIOC
 #define GPIO1_F_PIN_GPIO_PORT          		 GPIOC
 #define GPIO2_F_PIN_GPIO_PORT          		 GPIOC
 #define CAM_RESET_F_PIN_GPIO_PORT           GPIOC   


 /*##################### I2C1 ###################################*/
 /* User can use this section to tailor I2Cx instance used and associated resources */
 /* Definition for I2C1 Pins */
 #define BSP_I2C1                        I2C1
 #define BSP_I2C1_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
 #define BSP_I2C1_CLK_DISABLE()          __HAL_RCC_I2C1_CLK_DISABLE()
 #define BSP_I2C1_FORCE_RESET()          __HAL_RCC_I2C1_FORCE_RESET()
 #define BSP_I2C1_RELEASE_RESET()        __HAL_RCC_I2C1_RELEASE_RESET()
 #define BSP_I2C1_GPIO_PORT              GPIOB      /* GPIOB */
 #define BSP_I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
 #define BSP_I2C1_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
 #define BSP_I2C1_SCL_SDA_AF             GPIO_AF4_I2C1

 /* Maximum Timeout values for flags waiting loops. These timeouts are not based
    on accurate values, they just guarantee that the application will not remain
    stuck if the I2C communication is corrupted.
    You may modify these timeout values depending on CPU frequency and application
    conditions (interrupts routines ...). */
 #define BSP_I2C1_TIMEOUT_MAX            1000

 /* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
 /* Set 0x40E03E53 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
 #define I2C1_TIMING                     0x40E03E53

 /* Definition for I2Cx's NVIC */
 #define BSP_I2C1_EV_IRQn                    I2C1_EV_IRQn
 #define BSP_I2C1_ER_IRQn                    I2C1_ER_IRQn
 #define BSP_I2C1_EV_IRQHandler              I2C1_EV_IRQHandler
 #define BSP_I2C1_ER_IRQHandler              I2C1_ER_IRQHandler

 /*##################### I2C2 ###################################*/
  /* User can use this section to tailor I2Cx instance used and associated resources */
  /* Definition for I2C2 Pins */
  #define BSP_I2C2                        I2C2
  #define BSP_I2C2_CLK_ENABLE()           __HAL_RCC_I2C2_CLK_ENABLE()
  #define BSP_I2C2_CLK_DISABLE()          __HAL_RCC_I2C2_CLK_DISABLE()
  #define BSP_I2C2_FORCE_RESET()          __HAL_RCC_I2C2_FORCE_RESET()
  #define BSP_I2C2_RELEASE_RESET()        __HAL_RCC_I2C2_RELEASE_RESET()
  #define BSP_I2C2_GPIO_PORT              GPIOB      /* GPIOB */
  #define BSP_I2C2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
  #define BSP_I2C2_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
  #define BSP_I2C2_SCL_SDA_AF             GPIO_AF4_I2C2

  /* Maximum Timeout values for flags waiting loops. These timeouts are not based
     on accurate values, they just guarantee that the application will not remain
     stuck if the I2C communication is corrupted.
     You may modify these timeout values depending on CPU frequency and application
     conditions (interrupts routines ...). */
  #define BSP_I2C2_TIMEOUT_MAX            1000

  /* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
  /* Set 0x40E03E53 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
  #define I2C2_TIMING                     0x40E03E53

 /* Definition for I2Cx's NVIC */
 #define BSP_I2C2_EV_IRQn                    I2C2_EV_IRQn
 #define BSP_I2C2_ER_IRQn                    I2C2_ER_IRQn
 #define BSP_I2C2_EV_IRQHandler              I2C2_EV_IRQHandler
 #define BSP_I2C2_ER_IRQHandler              I2C2_ER_IRQHandler
 
 /*##################### USART1 ###################################*/
 /* communicate with phone */
 #define BSP_UART1                           USART1
 #define BSP_UART1_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
 #define BSP_UART1_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BSP_UART1_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BSP_UART1_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
 #define BSP_UART1_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

 /* Definition for ACC_UART3 Pins */
 
 #define BSP_UART1_TX_GPIO_PORT              GPIOA
 #define BSP_UART1_TX_AF                     GPIO_AF7_USART1 
 #define BSP_UART1_RX_GPIO_PORT              GPIOA
 #define BSP_UART1_RX_AF                     GPIO_AF7_USART1

 /* Definition for BSP_UART1's NVIC */
 #define BSP_UART1_IRQn                      USART1_IRQn
 #define BSP_UART1_IRQHandler                USART1_IRQHandler

 /* Size of Trasmission buffer */
 #define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
 /* Size of Reception buffer */
// #define RXBUFFERSIZE                      TXBUFFERSIZE

/*##################### USART2 ###################################*/
 /* communicate with phone */
 #define BSP_UART2                           USART2
 #define BSP_UART2_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
 #define BSP_UART2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BSP_UART2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BSP_UART2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
 #define BSP_UART2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

 /* Definition for ACC_UART3 Pins */
 
 #define BSP_UART2_TX_GPIO_PORT              GPIOA
 #define BSP_UART2_TX_AF                     GPIO_AF7_USART2 
 #define BSP_UART2_RX_GPIO_PORT              GPIOA
 #define BSP_UART2_RX_AF                     GPIO_AF7_USART2

 /* Definition for BSP_UART2's NVIC */
 #define BSP_UART2_IRQn                      USART2_IRQn
 #define BSP_UART2_IRQHandler                USART2_IRQHandler

 /* Size of Trasmission buffer */
 #define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
 /* Size of Reception buffer */
// #define RXBUFFERSIZE                      TXBUFFERSIZE

 /*##################### USART3 ###################################*/
 /* communicate with phone */
 #define BSP_UART3                           USART3
 #define BSP_UART3_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE()
 #define BSP_UART3_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
 #define BSP_UART3_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
 #define BSP_UART3_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
 #define BSP_UART3_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

 /* Definition for ACC_UART3 Pins */
 
 #define BSP_UART3_TX_GPIO_PORT              GPIOC
 #define BSP_UART3_TX_AF                     GPIO_AF7_USART3 
 #define BSP_UART3_RX_GPIO_PORT              GPIOC
 #define BSP_UART3_RX_AF                     GPIO_AF7_USART3

 /* Definition for BSP_UART3's NVIC */
 #define BSP_UART3_IRQn                      USART3_IRQn
 #define BSP_UART3_IRQHandler                USART3_IRQHandler

 /* Size of Trasmission buffer */
 #define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
 /* Size of Reception buffer */
// #define RXBUFFERSIZE                      TXBUFFERSIZE

/*##################### LED ###################################*/
 typedef enum
 {
   LED1 = 0,
   LED2,
   LED3,
   LED4
 } Led_TypeDef;


 #define LEDn                               4

 #define LED1_GPIO_PORT                     GPIOB
 #define LED1_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
 #define LED1_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()

 #define LED2_GPIO_PORT                     GPIOB
 #define LED2_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
 #define LED2_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()

 #define LED3_GPIO_PORT                     GPIOB
 #define LED3_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()  
 #define LED3_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()  

 #define LED4_GPIO_PORT                     GPIOC
 #define LED4_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()
 #define LED4_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOC_CLK_DISABLE()

 #define LEDx_GPIO_CLK_ENABLE()    do {LED1_GPIO_CLK_ENABLE(); LED2_GPIO_CLK_ENABLE();LED3_GPIO_CLK_ENABLE();LED4_GPIO_CLK_ENABLE();} while(0)
 #define LEDx_GPIO_CLK_DISABLE()   do {LED1_GPIO_CLK_DISABLE();LED2_GPIO_CLK_DISABLE();LED3_GPIO_CLK_DISABLE();LED4_GPIO_CLK_DISABLE();)while(0)

 /*##################### OUTPUT GPIO ###################################*/

 /*##################### PA ###################################*/
 #define SMB_SUSP_PIN_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
 #define SMB_SUSP_PIN_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()
 
 #define SMB_RESET_PIN_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
 #define SMB_RESET_PIN_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()
 
 #define BOOST_ENABLE_PIN_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BOOST_ENABLE_PIN_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE() 
 
 #define BOOST_9V_EN_PIN_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
 #define BOOST_9V_EN_PIN_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE() 
 
 #define CHG_PATH_EN_PIN_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
 #define CHG_PATH_EN_PIN_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE() 
 
 #define SMB_LPMODE_PIN_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
 #define SMB_LPMODE_PIN_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
 
  /*##################### PB ###################################*/ 
 #define PMUX1_CTRL_PIN_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
 #define PMUX1_CTRL_PIN_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE() 
 
 #define PMUX1_MODE_PIN_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
 #define PMUX1_MODE_PIN_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE() 
 
 #define PMUX1_CHA_EN_PIN_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
 #define PMUX1_CHA_EN_PIN_CLK_DISABLE()      __HAL_RCC_GPIOB_CLK_DISABLE()  
 
 #define PMUX1_CHB_EN_PIN_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
 #define PMUX1_CHB_EN_PIN_CLK_DISABLE()      __HAL_RCC_GPIOB_CLK_DISABLE()
 
 /*##################### PC ###################################*/ 
 #define GPIO1_F_PIN_CLK_ENABLE()         		__HAL_RCC_GPIOC_CLK_ENABLE()
 #define GPIO1_F_PIN_CLK_DISABLE()        		__HAL_RCC_GPIOC_CLK_DISABLE() 
 
 #define GPIO2_F_PIN_CLK_ENABLE()         		__HAL_RCC_GPIOC_CLK_ENABLE()
 #define GPIO2_F_PIN_CLK_DISABLE()        		__HAL_RCC_GPIOC_CLK_DISABLE() 
 
 #define CAM_RESET_F_PIN_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
 #define CAM_RESET_F_PIN_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE() 
 
 
 
 /*##################### INPUT GPIO ###################################*/
 #define SMB_CC_STS_PIN_CLK_ENABLE()           	__HAL_RCC_GPIOB_CLK_ENABLE()
 #define SMB_CC_STS_PIN_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

 #define SMB_STAT_PIN_CLK_ENABLE()             	__HAL_RCC_GPIOB_CLK_ENABLE()
 #define SMB_STAT_PIN_CLK_DISABLE()            	__HAL_RCC_GPIOB_CLK_DISABLE()
 
 #define SMB_SYS_PIN_CLK_ENABLE()             	__HAL_RCC_GPIOB_CLK_ENABLE()
 #define SMB_SYS_PIN_CLK_DISABLE()            	__HAL_RCC_GPIOB_CLK_DISABLE()
 
 #define BOOST_OCP_INT_PIN_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
 #define BOOST_OCP_INT_PIN_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()
 
 #define KEY_PIN_CLK_ENABLE()             			__HAL_RCC_GPIOB_CLK_ENABLE()
 #define KEY_PIN_CLK_DISABLE()            			__HAL_RCC_GPIOB_CLK_DISABLE()	 
	 
 #define FG_INT_PIN_CLK_ENABLE()             		__HAL_RCC_GPIOB_CLK_ENABLE()
 #define FG_INT_PIN_CLK_DISABLE()            		__HAL_RCC_GPIOB_CLK_DISABLE()	 
	
 #define PHONE_ATTACHED_PIN_CLK_ENABLE()      	__HAL_RCC_GPIOC_CLK_ENABLE()
 #define PHONE_ATTACHED_PIN_CLK_DISABLE()     	__HAL_RCC_GPIOC_CLK_DISABLE()
	 
/*##################### ADC ###################################*/
/* Definition for ADCx clock resources */
#define BSP_ADC1                            ADC1
#define BSP_ADC1_CLK_ENABLE()               __HAL_RCC_ADC_CLK_ENABLE()
#define BSP_ADC1_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define DMAx_CHANNELx_CLK_ENABLE()      		__HAL_RCC_DMA1_CLK_ENABLE()

#define BSP_ADC1_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define BSP_ADC1_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for BSP_ADC1 Channel Pin */
#define BSP_ADC1_CHANNEL_PIN_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define BSP_ADC1_CHANNEL_GPIO_PORT          GPIOA

/* Definition for BSP_ADC1's Channel */
#define USB_IN_ADC1_CHANNEL             ADC_CHANNEL_9
#define EXT_PWR_ADC1_CHANNEL            ADC_CHANNEL_11
#define ACC_ID_ADC1_CHANNEL            	ADC_CHANNEL_6

/* Definition for charging control GPIO */
typedef struct
{
	GPIO_PinState LPMODE_EN;
	GPIO_PinState SUSP;
	GPIO_PinState RST;
}SMB_IO_Ctrl;	 	 

typedef struct
{
	GPIO_PinState MODE;
	GPIO_PinState CTRL;
	GPIO_PinState CHA_EN;
	GPIO_PinState CHB_EN;	
}PMUX1_IO_Ctrl;

typedef struct
{
	GPIO_PinState ENABLE;
	GPIO_PinState B9V_EN;
}BOOST_IO_Ctrl;

typedef struct
{
	SMB_IO_Ctrl 	SMB;
	PMUX1_IO_Ctrl PMUX1;
	BOOST_IO_Ctrl BOOST;
	GPIO_PinState CHG_PATH_EN;
}PB_IO;


/** @defgroup Power pack_Exported_Functions Exported Functions
  * @{
  */

/**
  * @}
  */ 
/* Get firmware revision function */
/* Link function for firmware revision */
uint32_t         		BSP_GetVersion(void);
/**
  * @}
  */ 
/* LED function */
/* Link function for LED */
void             		BSP_LED_Init(void);
void             		BSP_LED_On(Led_TypeDef Led);
void             		BSP_LED_Off(Led_TypeDef Led);
void             		BSP_LED_Toggle(Led_TypeDef Led);
void 						 		BSP_LED_Light(uint8_t Led);
/**
  * @}
  */ 
/* I2C1 bus function */
/* Link function for I2C peripherals */
void               	BSP_I2C1_Init(void);
void               	BSP_I2C1_Error (void);
void               	BSP_I2C1_MspInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef	 	BSP_I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize, uint8_t Value);
uint8_t 					 	BSP_I2C1_Read(uint8_t Addr, uint16_t Reg ,uint16_t RegAddSize);
HAL_StatusTypeDef  	BSP_I2C1_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  	BSP_I2C1_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  	BSP_I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
/**
  * @}
  */ 
/* I2C2 bus function */
/* Link function for I2C peripherals */
void               	BSP_I2C2_Init(void);
void               	BSP_I2C2_Error (void);
void               	BSP_I2C2_MspInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef	 	BSP_I2C2_Write(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize, uint8_t Value);
uint8_t 					 	BSP_I2C2_Read(uint8_t Addr, uint16_t Reg,uint16_t RegAddSize);
HAL_StatusTypeDef  	BSP_I2C2_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  	BSP_I2C2_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
HAL_StatusTypeDef  	BSP_I2C2_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
/**
  * @}
  */ 
/* UART3 bus function */
/* Link function for UART peripherals */
void 								BSP_UART3_Init(void);
/**
  * @}
  */ 
/* ADC1 bus function */
/* Link function for UART peripherals */
void 								BSP_ADC1_Init(void);
/**
  * @}
  */ 
/* EXTI function */
/* Link function for EXTI  */
static void 				BSP_SMB_CC_IRQHandler_Config(void);
static void 				BSP_SMB_STAT_IRQHandler_Config(void);
static void 				BSP_SMB_SYSOK_IRQHandler_Config(void);
static void 				BSP_BOOST_OVP_IRQHandler_Config(void);
static void 				BSP_KEY_IRQHandler_Config(void);
static void 				BSP_EXIT15_10_IRQHandler_Config(void);
/**
  * @}
  */ 
/* Charging path function */
/* Link function for setting charging path   */
void 								BSP_IO_Config(PB_IO *io_ctrl);
void 								BSP_STANDBY(void);
void 								BSP_USB2PACK(void);
void 								BSP_USB2PHONE(void);
void 								BSP_USB2BOTH(void);
void 								BSP_DCIN2PACK(void);
void 								BSP_DCIN2PHONE(void);
void 								BSP_DCIN2BOTH(void);
void 								BSP_BOOST2PHONE(void);
void 								BSP_PHONE2PACK(void);
/**
  * @}
  */ 
/* Hardware init function */
/* Link function for hardware initialization */
void 								BSP_OUTPUT_GPIO_Init(void);
void 							 	BSP_POWER_PACK_Init(void);

    
#ifdef __cplusplus
}
#endif

#endif /* __POWER_PACK_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

