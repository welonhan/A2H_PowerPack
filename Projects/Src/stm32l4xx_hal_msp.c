/**
  ******************************************************************************
  * @file    I2C/I2C_WakeUpFromStop/Src/stm32l4xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "power_pack.h"
//#include "stm32l4xx_hal.h"

extern DMA_HandleTypeDef         		DmaHandle;
/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  if(hi2c->Instance==BSP_I2C2)
	{
	/*##-1- Set source clock to SYSCLK for I2C2 ################################################*/
		RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
		RCC_PeriphCLKInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

		/*##-2- Configure the GPIOs ################################################*/

		/* Enable GPIO clock */
		BSP_I2C2_GPIO_CLK_ENABLE();

		/* Configure I2C SCL & SDA as alternate function  */
		GPIO_InitStruct.Pin       = (BSP_I2C2_SCL_PIN);
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = BSP_I2C2_SCL_SDA_AF;
		HAL_GPIO_Init(BSP_I2C2_GPIO_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin       = (BSP_I2C2_SDA_PIN);
		HAL_GPIO_Init(BSP_I2C2_GPIO_PORT, &GPIO_InitStruct);
		/*##-3- Configure the Eval I2C peripheral #######################################*/
		/* Enable I2C clock */
		BSP_I2C2_CLK_ENABLE();
	}
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==BSP_I2C2)
	{
		/*##-1- Reset peripherals ##################################################*/
		BSP_I2C2_FORCE_RESET();
		BSP_I2C2_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure I2C Tx as alternate function  */
		HAL_GPIO_DeInit(BSP_I2C2_GPIO_PORT, BSP_I2C2_SCL_PIN);
		/* Configure I2C Rx as alternate function  */
		HAL_GPIO_DeInit(BSP_I2C2_GPIO_PORT, BSP_I2C2_SDA_PIN);

		/*##-3- Disable the NVIC for I2C ##########################################*/
		//HAL_NVIC_DisableIRQ(BSP_I2C2_ER_IRQn);
		//HAL_NVIC_DisableIRQ(BSP_I2C2_EV_IRQn);
	}

}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;  	

  if(huart->Instance==BSP_UART3)
  {
		/*##-1- Set source clock to SYSCLK for I2C1 ################################################*/
		RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
		RCC_PeriphCLKInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
			
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		BSP_UART3_TX_GPIO_CLK_ENABLE();
		BSP_UART3_RX_GPIO_CLK_ENABLE();

		/* Enable BSP_UART3 clock */
		BSP_UART3_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_UART3_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = BSP_UART3_TX_AF;

		HAL_GPIO_Init(BSP_UART3_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = BSP_UART3_RX_PIN;
		GPIO_InitStruct.Alternate = BSP_UART3_RX_AF;

		HAL_GPIO_Init(BSP_UART3_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority(BSP_UART3_IRQn, 13, 0);
		HAL_NVIC_EnableIRQ(BSP_UART3_IRQn);
  }
	else if(huart->Instance==BSP_UART2)
  {
		/*##-1- Set source clock to SYSCLK for I2C1 ################################################*/
		RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
		RCC_PeriphCLKInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
			
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		BSP_UART2_TX_GPIO_CLK_ENABLE();
		BSP_UART2_RX_GPIO_CLK_ENABLE();

		/* Enable BSP_UART3 clock */
		BSP_UART2_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_UART2_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = BSP_UART2_TX_AF;

		HAL_GPIO_Init(BSP_UART2_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = BSP_UART2_RX_PIN;
		GPIO_InitStruct.Alternate = BSP_UART2_RX_AF;

		HAL_GPIO_Init(BSP_UART2_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority(BSP_UART2_IRQn, 13, 0);
		HAL_NVIC_EnableIRQ(BSP_UART2_IRQn);
  }
	else if(huart->Instance==BSP_UART1)
  {
		/*##-1- Set source clock to SYSCLK for I2C1 ################################################*/
		RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
		RCC_PeriphCLKInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
			
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		BSP_UART1_TX_GPIO_CLK_ENABLE();
		BSP_UART1_RX_GPIO_CLK_ENABLE();

		/* Enable BSP_UART1 clock */
		BSP_UART1_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ##########################################*/
		/* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = BSP_UART1_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLUP;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = BSP_UART1_TX_AF;

		HAL_GPIO_Init(BSP_UART1_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */		
		GPIO_InitStruct.Pin = BSP_UART1_RX_PIN;
		GPIO_InitStruct.Alternate = BSP_UART1_RX_AF;

		HAL_GPIO_Init(BSP_UART1_RX_GPIO_PORT, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
		HAL_NVIC_SetPriority(BSP_UART1_IRQn, 13, 0);
		HAL_NVIC_EnableIRQ(BSP_UART1_IRQn);
  }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if(huart->Instance==BSP_UART3)
  {
		/*##-1- Reset peripherals ##################################################*/
		BSP_UART3_FORCE_RESET();
		BSP_UART3_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART3_TX_GPIO_PORT, BSP_UART3_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART3_RX_GPIO_PORT, BSP_UART3_RX_PIN);

		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(BSP_UART3_IRQn);
  }
	else if(huart->Instance==BSP_UART2)
  {
		/*##-1- Reset peripherals ##################################################*/
		BSP_UART2_FORCE_RESET();
		BSP_UART2_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART2_TX_GPIO_PORT, BSP_UART2_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART2_RX_GPIO_PORT, BSP_UART2_RX_PIN);

		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(BSP_UART2_IRQn);
  }
	else if(huart->Instance==BSP_UART1)
  {
		/*##-1- Reset peripherals ##################################################*/
		BSP_UART1_FORCE_RESET();
		BSP_UART1_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks #################################*/
		/* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART1_TX_GPIO_PORT, BSP_UART1_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(BSP_UART1_RX_GPIO_PORT, BSP_UART1_RX_PIN);

		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(BSP_UART1_IRQn);
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  //DMA_HandleTypeDef         		DmaHandle;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* ADC Periph clock enable */
  BSP_ADC1_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  /* Enable DMA clock */
  DMAx_CHANNELx_CLK_ENABLE();

  /*##- 2- Configure peripheral GPIO #########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = ACC_ID_ADC1_IN6_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_ADC1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = USB_IN_ADC1_IN9_PIN;
	HAL_GPIO_Init(BSP_ADC1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = EXT_PWR_ADC1_IN11_PIN;
	HAL_GPIO_Init(BSP_ADC1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
	
	
  /*##- 3- Configure DMA #####################################################*/

  /*********************** Configure DMA parameters ***************************/
  DmaHandle.Instance                 = DMA1_Channel1;
  DmaHandle.Init.Request             = DMA_REQUEST_0;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  DmaHandle.Init.Mode                = DMA_CIRCULAR;
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the DMA handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

  /* NVIC configuration for DMA Input data interrupt */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0x0E,0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

  /*##-1- Reset peripherals ##################################################*/
  BSP_ADC1_FORCE_RESET();
  BSP_ADC1_RELEASE_RESET();
  /* ADC Periph clock disable
   (automatically reset all ADC's) */
  __HAL_RCC_ADC_CLK_DISABLE();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(BSP_ADC1_CHANNEL_GPIO_PORT, ACC_ID_ADC1_IN6_PIN);
	HAL_GPIO_DeInit(BSP_ADC1_CHANNEL_GPIO_PORT, USB_IN_ADC1_IN9_PIN);
	HAL_GPIO_DeInit(BSP_ADC1_CHANNEL_GPIO_PORT, EXT_PWR_ADC1_IN11_PIN);
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
