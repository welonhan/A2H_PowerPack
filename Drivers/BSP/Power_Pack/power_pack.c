/* Includes ------------------------------------------------------------------*/
#include "power_pack.h"

#include <stdio.h>

const uint16_t 	LED_PIN[LEDn]={LED1_PIN,LED2_PIN,LED3_PIN,LED4_PIN};
GPIO_TypeDef* 	LED_PORT[LEDn]={LED1_GPIO_PORT,LED2_GPIO_PORT,LED3_GPIO_PORT,LED4_GPIO_PORT};

extern uint32_t I2c1Timeout;    /*<! Value of Timeout when I2C1 communication fails */
extern uint32_t I2c2Timeout;    /*<! Value of Timeout when I2C1 communication fails */
extern I2C_HandleTypeDef powerpack_I2c1,powerpack_I2c2;

extern ADC_HandleTypeDef 						AdcHandle;
extern ADC_ChannelConfTypeDef       sConfig;
extern uint16_t   aADCxConvertedData[4];

extern UART_HandleTypeDef Uart1Handle,Uart2Handle,Uart3Handle;
/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *            @arg  
  * @retval None
  */
void BSP_LED_Init(void)
{
	uint8_t Led;
  GPIO_InitTypeDef  GPIO_InitStruct;
  

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE();

	for(Led=0;Led<LEDn;Led++)
	{
		/* Configure the GPIO_LED pin */
		GPIO_InitStruct.Pin = LED_PIN[Led];
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
		HAL_GPIO_Init(LED_PORT[Led], &GPIO_InitStruct);
		HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
	}
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  LED1/2/3/4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *            @arg  LED1/2/3/4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED1/2/3/4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Turn on Led1-Led4 in binary code
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  1-15
	*			@arg	led1 	led2 	led3 	led4
	*			0			0			0			0			0	
	*			1			1			0			0			0
	*			15		1			1			1			1
  * @retval None
  */
void 	BSP_LED_Light(uint8_t Led)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		if(Led%2)
			BSP_LED_On((Led_TypeDef) i);
		else
			BSP_LED_Off((Led_TypeDef) i);
		Led = Led >>1;
	}
}

/******************************* I2C1 Routines *********************************/

/**
  * @brief I2C Bus initialization
  * @retval None
  */
void BSP_I2C1_Init(void)
{
  if(HAL_I2C_GetState(&powerpack_I2c1) == HAL_I2C_STATE_RESET)
  {
    powerpack_I2c1.Instance              = BSP_I2C1;
    powerpack_I2c1.Init.Timing           = I2C1_TIMING;
    powerpack_I2c1.Init.OwnAddress1      = 0;
    powerpack_I2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    powerpack_I2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    powerpack_I2c1.Init.OwnAddress2      = 0;
    powerpack_I2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    powerpack_I2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    powerpack_I2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  

    /* Init the I2C */
    HAL_I2C_Init(&powerpack_I2c1);
  }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
	*	@param  RegAddSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  Value: Data to be written
  * @retval None
  */
HAL_StatusTypeDef BSP_I2C1_Write(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&powerpack_I2c1, Addr, (uint16_t)Reg, RegAddSize, &Value, 1, I2c1Timeout); 

  return status;
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
	*	@param  RegAddSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @retval Read data
  */
uint8_t BSP_I2C1_Read(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(&powerpack_I2c1, Addr, Reg, RegAddSize, &Value, 1, I2c1Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    BSP_I2C1_Error();
  }
  return Value;   
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr  : I2C Address
  * @param  Reg   : Reg Address 
  * @param  RegSize : I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  pBuffer : pointer to read data buffer
  * @param  Length : length of the data
  * @retval 0 if no problems to read multiple data
  */
HAL_StatusTypeDef BSP_I2C1_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&powerpack_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2c1Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    BSP_I2C1_Error();
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef BSP_I2C1_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&powerpack_I2c1, DevAddress, Trials, I2c1Timeout));
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  RegSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  pBuffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval None
  */
HAL_StatusTypeDef BSP_I2C1_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&powerpack_I2c1, Addr, Reg, RegSize, pBuffer, Length, I2c1Timeout); 
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    BSP_I2C1_Error();
  }        
  return status;
}

/**
  * @brief  Manages error callback to re-initialize I2C.
  * @retval None
  */
void BSP_I2C1_Error(void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&powerpack_I2c1);
  
  /* Re-Initiaize the I2C communication BUS */
  BSP_I2C1_Init();
}


/******************************* I2C2 Routines *********************************/

/**
  * @brief I2C Bus initialization
  * @retval None
  */
void BSP_I2C2_Init(void)
{
  if(HAL_I2C_GetState(&powerpack_I2c2) == HAL_I2C_STATE_RESET)
  {
    powerpack_I2c2.Instance              = BSP_I2C2;
    powerpack_I2c2.Init.Timing           = I2C2_TIMING;
    powerpack_I2c2.Init.OwnAddress1      = 0;
    powerpack_I2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    powerpack_I2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    powerpack_I2c2.Init.OwnAddress2      = 0;
    powerpack_I2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    powerpack_I2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    powerpack_I2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  

    /* Init the I2C */
    HAL_I2C_Init(&powerpack_I2c2);
  }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
	*	@param  RegAddSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  Value: Data to be written
  * @retval None
  */
HAL_StatusTypeDef BSP_I2C2_Write(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&powerpack_I2c2, Addr, (uint16_t)Reg, RegAddSize, &Value, 1, I2c2Timeout); 
	
	return status;  
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
	*	@param  RegAddSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @retval Read data
  */
uint8_t BSP_I2C2_Read(uint8_t Addr, uint16_t Reg, uint16_t RegAddSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(&powerpack_I2c2, Addr, Reg, RegAddSize, &Value, 1, I2c2Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    BSP_I2C2_Error();
  }
  return Value;   
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr  : I2C Address
  * @param  Reg   : Reg Address 
  * @param  RegSize : I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  pBuffer : pointer to read data buffer
  * @param  Length : length of the data
  * @retval 0 if no problems to read multiple data
  */
HAL_StatusTypeDef BSP_I2C2_ReadBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&powerpack_I2c2, Addr, Reg, RegSize, pBuffer, Length, I2c2Timeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    BSP_I2C2_Error();
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef BSP_I2C2_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&powerpack_I2c2, DevAddress, Trials, I2c2Timeout));
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  RegSize: I2C_MEMADD_SIZE_8BIT , I2C_MEMADD_SIZE_16BIT 
  * @param  pBuffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval None
  */
HAL_StatusTypeDef BSP_I2C2_WriteBuffer(uint16_t Addr, uint16_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&powerpack_I2c2, Addr, Reg, RegSize, pBuffer, Length, I2c2Timeout); 
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    BSP_I2C2_Error();
  }        
  return status;
}

/**
  * @brief  Manages error callback to re-initialize I2C.
  * @retval None
  */
void BSP_I2C2_Error(void)
{
  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&powerpack_I2c2);
  
  /* Re-Initiaize the I2C communication BUS */
  BSP_I2C2_Init();
}

/******************************* UART1 Routines *********************************/

/**
  * @brief  UART1 initialization.
  * @retval None
  */
void BSP_UART1_Init(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  Uart1Handle.Instance        = BSP_UART1;

  Uart1Handle.Init.BaudRate   = 115200;
  Uart1Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Uart1Handle.Init.StopBits   = UART_STOPBITS_1;
  Uart1Handle.Init.Parity     = UART_PARITY_NONE;
  Uart1Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  Uart1Handle.Init.Mode       = UART_MODE_TX_RX;
	//UARTHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  //UARTHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	
  if(HAL_UART_DeInit(&Uart1Handle) != HAL_OK)
  {
    //Error_Handler();
  }  
  if(HAL_UART_Init(&Uart1Handle) != HAL_OK)
  {
   // printf("Uart1 init error\n\r");
  }	
}

/******************************* UART2 Routines *********************************/

/**
  * @brief  UART2 initialization.
  * @retval None
  */
void BSP_UART2_Init(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  Uart2Handle.Instance        = BSP_UART2;

  Uart2Handle.Init.BaudRate   = 115200;
  Uart2Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Uart2Handle.Init.StopBits   = UART_STOPBITS_1;
  Uart2Handle.Init.Parity     = UART_PARITY_NONE;
  Uart2Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  Uart2Handle.Init.Mode       = UART_MODE_TX_RX;
	//UARTHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  //UARTHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	
  if(HAL_UART_DeInit(&Uart2Handle) != HAL_OK)
  {
    //Error_Handler();
  }  
  if(HAL_UART_Init(&Uart2Handle) != HAL_OK)
  {
   // printf("Uart2 init error\n\r");
  }	
}

/******************************* UART3 Routines *********************************/

/**
  * @brief  UART3 initialization.
  * @retval None
  */
void BSP_UART3_Init(void)
{
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  Uart3Handle.Instance        = BSP_UART3;

  Uart3Handle.Init.BaudRate   = 115200;
  Uart3Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Uart3Handle.Init.StopBits   = UART_STOPBITS_1;
  Uart3Handle.Init.Parity     = UART_PARITY_NONE;
  Uart3Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  Uart3Handle.Init.Mode       = UART_MODE_TX_RX;
	//UARTHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  //UARTHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	
  if(HAL_UART_DeInit(&Uart3Handle) != HAL_OK)
  {
    //Error_Handler();
  }  
  if(HAL_UART_Init(&Uart3Handle) != HAL_OK)
  {
    //printf("uart3 init error\n\r");
  }	
}

/**
  * @brief  ADC1 initialization
  * @retval None
  */
void BSP_ADC1_Init(void)
{
	//ADC_HandleTypeDef             AdcHandle;
	//ADC_ChannelConfTypeDef        sConfig;
	
/* ### - 1 - Initialize ADC peripheral #################################### */
  AdcHandle.Instance          = BSP_ADC1;
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    //Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* Synchronous clock mode, input ADC clock divided by 2*/
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = ENABLE;                      	/* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;           		/* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  AdcHandle.Init.NbrOfConversion       = 4;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* ADC DMA continuous request to match with DMA circular mode */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_Init fail\n\r");
  }
  
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
   // Error_Handler();
		printf("HAL_ADCEx_Calibration_Start fail\n\r");
		
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  sConfig.Channel      = USB_IN_ADC1_CHANNEL;         /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = LL_ADC_SAMPLINGTIME_47CYCLES_5;   	/* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_ConfigChannel fail:USB IN\n\r");
  }
  
	sConfig.Channel      = EXT_PWR_ADC1_CHANNEL;          /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_2;           /* Rank of sampled channel number ADCx_CHANNEL */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_ConfigChannel fail:DC IN\n\r");
  }
	sConfig.Channel      = ACC_ID_ADC1_CHANNEL;          /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_3;           /* Rank of sampled channel number ADCx_CHANNEL */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_ConfigChannel fail:CHG OUT\n\r");
  }
	sConfig.Channel    = ADC_CHANNEL_VREFINT;          /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_4;           /* Rank of sampled channel number ADCx_CHANNEL */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_ConfigChannel fail:VREFINT\n\r");
  }
	
  /* ### - 4 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&AdcHandle,
                        (uint32_t *)aADCxConvertedData,
                        4
                       ) != HAL_OK)
  {
    //Error_Handler();
		printf("HAL_ADC_Start_DMA fail\n\r");
  }
	
}


/******************************* EXIT Routines *********************************/

void BSP_SMB_CC_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	SMB_CC_STS_PIN_CLK_ENABLE();
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  
	
	GPIO_InitStruct.Pin = SMB_CC_STS_PIN;
  HAL_GPIO_Init(SMB_CC_STS_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  //HAL_NVIC_SetPriority((IRQn_Type)EXTI0_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn));
}

void BSP_SMB_STAT_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	SMB_STAT_PIN_CLK_ENABLE() ;
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  
	
	GPIO_InitStruct.Pin = SMB_STAT_PIN;
  HAL_GPIO_Init(SMB_STAT_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 10 ,0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI1_IRQn));
}

void BSP_SMB_SYSOK_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	SMB_SYS_PIN_CLK_ENABLE();
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  
	
	GPIO_InitStruct.Pin = SMB_SYS_PIN;
  HAL_GPIO_Init(SMB_SYS_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  //HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 5, 0);
	//HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI2_IRQn));
}

void BSP_BOOST_OVP_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	BOOST_OCP_INT_PIN_CLK_ENABLE() ;
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  
	
	GPIO_InitStruct.Pin = BOOST_OCP_INT_PIN;
  HAL_GPIO_Init(BOOST_OCP_INT_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  //HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 6, 0);
	//HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI3_IRQn));
}

void BSP_ATTACH_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	PHONE_ATTACHED_PIN_CLK_ENABLE() ;
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  
	
	GPIO_InitStruct.Pin = PHONE_ATTACHED_PIN;
  HAL_GPIO_Init(PHONE_ATTACHED_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI4_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI4_IRQn));
}


void BSP_KEY_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	KEY_PIN_CLK_ENABLE() ;
	  
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  
	
	GPIO_InitStruct.Pin = KEY_PIN;
  HAL_GPIO_Init(KEY_PIN_GPIO_PORT, &GPIO_InitStruct);	 
  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI9_5_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));
}

void	BSP_EXIT15_10_IRQHandler_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	FG_INT_PIN_CLK_ENABLE() ;
	
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  
	
	GPIO_InitStruct.Pin = FG_INT_PIN;
  HAL_GPIO_Init(FG_INT_PIN_PIN_GPIO_PORT, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI15_10_IRQn, 12, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
}

/******************************* GPIO control Routines *********************************/
void BSP_IO_Config(PB_IO *io_ctrl)
{
	HAL_GPIO_WritePin(SMB_LPMODE_EN_PIN_GPIO_PORT, 	SMB_LPMODE_EN_PIN, 	io_ctrl->SMB.LPMODE_EN);
	HAL_GPIO_WritePin(SMB_SUSP_PIN_GPIO_PORT, 			SMB_SUSP_PIN, 			io_ctrl->SMB.SUSP	);
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, 			SMB_RESET_PIN, 			io_ctrl->SMB.RST);
		
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		io_ctrl->PMUX1.MODE);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		io_ctrl->PMUX1.CTRL);
	HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		io_ctrl->PMUX1.CHA_EN);
	HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		io_ctrl->PMUX1.CHB_EN);	
	
	HAL_GPIO_WritePin(BOOST_ENABLE_PIN_GPIO_PORT, 	BOOST_ENABLE_PIN,		io_ctrl->BOOST.ENABLE);
	HAL_GPIO_WritePin(BOOST_9V_EN_PIN_GPIO_PORT, 		BOOST_9V_EN_PIN,		io_ctrl->BOOST.B9V_EN);
	
	HAL_GPIO_WritePin(CHG_PATH_EN_PIN_GPIO_PORT, 		CHG_PATH_EN_PIN,		io_ctrl->CHG_PATH_EN);
}

void BSP_STANDBY(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_RESET;	
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_RESET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_RESET;
	
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_RESET;
	BSP_IO_Config(&io_ctrl);	
}

void BSP_USB2PACK(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_RESET;	
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_RESET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_RESET;
	
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_RESET;
	BSP_IO_Config(&io_ctrl);	
}

void BSP_USB2BOTH(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_RESET;
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_RESET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_RESET;
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_SET;
	BSP_IO_Config(&io_ctrl);	
}

void BSP_USB2PHONE(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_RESET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_RESET;	
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_RESET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_RESET;
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_SET;
	BSP_IO_Config(&io_ctrl);	
}

void BSP_BOOST2PHONE(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_RESET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_RESET;	
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_SET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_SET;
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_RESET;
	
	
	BSP_IO_Config(&io_ctrl);	
}

void BSP_PHONE2PACK(void)
{
	PB_IO io_ctrl;
	
	io_ctrl.SMB.LPMODE_EN				=GPIO_PIN_SET;
	io_ctrl.SMB.SUSP						=GPIO_PIN_SET;
	io_ctrl.SMB.RST							=GPIO_PIN_SET;	
	
	io_ctrl.PMUX1.MODE					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CTRL					=GPIO_PIN_SET;
	io_ctrl.PMUX1.CHA_EN				=GPIO_PIN_RESET;
	io_ctrl.PMUX1.CHB_EN				=GPIO_PIN_SET;	
	
	io_ctrl.BOOST.ENABLE				=GPIO_PIN_RESET;
	io_ctrl.BOOST.B9V_EN				=GPIO_PIN_RESET;
	io_ctrl.CHG_PATH_EN					=GPIO_PIN_RESET;
	BSP_IO_Config(&io_ctrl);	
}

/******************************* Hardware Init Routines *********************************/
void BSP_OUTPUT_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;  

  /* Enable the GPIO_LED Clock */
 	PMUX1_CTRL_PIN_CLK_ENABLE();
	PMUX1_MODE_PIN_CLK_ENABLE();	
	BOOST_ENABLE_PIN_CLK_ENABLE();
	BOOST_9V_EN_PIN_CLK_ENABLE();
	SMB_SUSP_PIN_CLK_ENABLE();
	SMB_RESET_PIN_CLK_ENABLE();
	SMB_LPMODE_PIN_CLK_ENABLE();	
	PMUX1_CHA_EN_PIN_CLK_ENABLE();
	PMUX1_CHB_EN_PIN_CLK_ENABLE();	
	CHG_PATH_EN_PIN_CLK_ENABLE();
	
	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
	
	
	GPIO_InitStruct.Pin =PMUX1_CTRL_PIN ;
	HAL_GPIO_Init(PMUX1_CTRL_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = PMUX1_MODE_PIN;
	HAL_GPIO_Init(PMUX1_MODE_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_RESET);	
	
	GPIO_InitStruct.Pin = BOOST_ENABLE_PIN;
	HAL_GPIO_Init(BOOST_ENABLE_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BOOST_ENABLE_PIN_GPIO_PORT, 	BOOST_ENABLE_PIN,		GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = BOOST_9V_EN_PIN;
	HAL_GPIO_Init(BOOST_9V_EN_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(BOOST_9V_EN_PIN_GPIO_PORT, 		BOOST_9V_EN_PIN,		GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = SMB_SUSP_PIN;
	HAL_GPIO_Init(SMB_SUSP_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SMB_SUSP_PIN_GPIO_PORT, 			SMB_SUSP_PIN, 			GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = SMB_RESET_PIN;
	HAL_GPIO_Init(SMB_RESET_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, 			SMB_RESET_PIN, 			GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = SMB_LPMODE_EN_PIN;
	HAL_GPIO_Init(SMB_LPMODE_EN_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SMB_LPMODE_EN_PIN_GPIO_PORT, 	SMB_LPMODE_EN_PIN, 	GPIO_PIN_RESET);	
	
	GPIO_InitStruct.Pin = PMUX1_CHA_EN_PIN;
	HAL_GPIO_Init(PMUX1_CHA_EN_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = PMUX1_CHB_EN_PIN;
	HAL_GPIO_Init(PMUX1_CHB_EN_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		GPIO_PIN_RESET);	
	
	GPIO_InitStruct.Pin = CHG_PATH_EN_PIN;
	HAL_GPIO_Init(CHG_PATH_EN_PIN_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(CHG_PATH_EN_PIN_GPIO_PORT, 		CHG_PATH_EN_PIN,		GPIO_PIN_RESET);
}

void BSP_POWER_PACK_Init(void)
{
	
	BSP_UART3_Init();	
	BSP_UART1_Init();
	BSP_UART2_Init();
	printf("\n\r\n\r\n\r\n\r");
	printf("******************************\n\r");
	printf("*        Power pack          *\n\r");
	printf("*                            *\n\r");
	printf("******************************\n\r");
	printf("UART1,2,3 init ok!\n\r");
	
	BSP_OUTPUT_GPIO_Init();
	printf("GPIO init ok!\n\r");
	
	BSP_STANDBY();
	printf("Setting GPIO to default!\n\r");
	
	BSP_LED_Init();
	printf("LED init ok!\n\r");
	
	BSP_I2C1_Init();
	BSP_I2C2_Init();
	printf("I2C init ok!\n\r");
	
	BSP_ADC1_Init();
	printf("ADC1 init ok!\n\r");
	
	BSP_SMB_CC_IRQHandler_Config();
 	BSP_SMB_STAT_IRQHandler_Config();
 	BSP_SMB_SYSOK_IRQHandler_Config();
 	BSP_BOOST_OVP_IRQHandler_Config();
	BSP_ATTACH_IRQHandler_Config();
 	BSP_KEY_IRQHandler_Config();
	printf("KEY init ok!\n\r");
	
 	BSP_EXIT15_10_IRQHandler_Config();
	
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
