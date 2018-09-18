/**
  ******************************************************************************
  * @file    I2C/I2C_WakeUpFromStop/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32L4xx I2C HAL API to transmit 
  *          and receive a data buffer with a communication process in stop mode
  *          based on IT transfer. 
  *          The communication is done using 2 Boards.
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
#include <stdio.h>
#include "string.h"

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define RXCOMMANDSIZE   	0x20
#define RXBUFFERSIZE   		0x20
#define LR_ASCII_VALUE 		((uint8_t)0x0A)
#define CR_ASCII_VALUE 		((uint8_t)0x0D)
#define SPACE_ASCII_VALUE ((uint8_t)0x20)
	
char RxCommand[RXCOMMANDSIZE];
char PbCommand[RXCOMMANDSIZE];
uint8_t RxBuffer[RXBUFFERSIZE] = {0}; //transmitting byte per byte

#define RXBUFFERSIZE_U1   		0x20
uint8_t RxBuffer_U1[RXBUFFERSIZE_U1] = {0}; //transmitting byte per byte

//#define RXBUFFERSIZE_U3   		0x20
//uint8_t RxBuffer_U3[RXBUFFERSIZE_U3] = {0}; //transmitting byte per byte
//#define TXBUFFERSIZE_U3   		0x20
//uint8_t TxBuffer_U3[TXBUFFERSIZE_U3] = {0}; //transmitting byte per byte

//#define I2C_TIMING      0x0020098E

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
uint16_t   										aADCxConvertedData[4];

UART_HandleTypeDef 						Uart1Handle,Uart2Handle,Uart3Handle;
__IO ITStatus 								UartReady = RESET;

DMA_HandleTypeDef         		DmaHandle;

IWDG_HandleTypeDef 						IwdgHandle;

WWDG_HandleTypeDef 						WwdgHandle;

CRC_HandleTypeDef 						CrcHandle;

I2C_HandleTypeDef 						powerpack_I2c1,powerpack_I2c2;
uint32_t 											I2c1Timeout = BSP_I2C1_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
uint32_t 											I2c2Timeout = BSP_I2C2_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */

uint32_t 											usb_in_voltage, ext_pwr_voltage, acc_id_voltage,vdda;

uint8_t 				uart1_packet_num, uart1_packet_size;

PACK_INFO 			pack_info;
SMB_TADC 				smb_tadc;

USB_TYPE 				*usb_type_temp;
SMB_IN_STATE 		in_state;
USB_TYPE 				usb_type;

uint16_t 				cw_vbat;	

PACK_STATUS 		pack_status=STATUS_STANDBY;
ACC_TypeDef 		acc_type;

uint16_t 				wdg_count=0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t UART1_TX_handle, UART1_RX_handle, UART1_RX_handle_pkt, UART2_handle, 
			UART3_TX_handle, UART3_RX_handle, ADC_handle, CHARGING_handle, SMB_handle,LED_handle,ATTACH_handle,WDG_handle;

EventGroupHandle_t xCreatedEventGroup;
static xSemaphoreHandle xSemaphore_uart1_int,xSemaphore_uart2_int,xSemaphore_uart3_int,xSemaphore_wwdg;
static xSemaphoreHandle xCountingSemaphore_smb, xCountingSemaphore_key, xCountingSemaphore_attach;
static xQueueHandle xQueue_pack_info, xQueue_uart1_tx, xQueue_uart1_rx_handle;
static xSemaphoreHandle xMutex_i2c;

UART1_TX_QUEUE tx_queue[3];


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
	//disable interrupt
	__set_PRIMASK(1);		
  
	HAL_Init();

	/* Configure the system clock to 80 MHz */
	SystemClock_Config();
	
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	
	BSP_POWER_PACK_Init();
		
	BSP_SMB_Enable();
	BSP_SMB_Init();
	
	if(BSP_CW_BAT_Init()==0)
		printf("CW2015 init ok!\n\r");
	
	pack_info.ATTACH=0;
	pack_info.USB=0;
	pack_info.DCIN=0;
	
	pack_info.PHONE_SOC=0;	
	pack_info.PHONE_USB_VOLTAGE=0;
	
	if(HAL_GPIO_ReadPin(PHONE_ATTACHED_PIN_GPIO_PORT,PHONE_ATTACHED_PIN))
	{
		pack_info.ATTACH=0;
		printf("Phone detached!!\n\r");
	}
	else
	{
		pack_info.ATTACH=1;
		printf("Phone attached!!\n\r");
	}
	
	BSP_SMB_INT_Type(&in_state);
	if(in_state.USBIN==1)
	{
		//BSP_SMB_USBIN_Exit_Suspend();
		pack_info.USB=1;			
	}
	else if(in_state.USBIN==0)
	{
		pack_info.USB=0;
		//BSP_SMB_USBIN_Suspend();
	}
	
	pack_info.PACK_SOC=BSP_CW_Get_Capacity(pack_info.USB);
	/*
	if(in_state.DCIN==1)
	{
		pack_info.PHONE_USB=1;		
	}
	else if(in_state.DCIN==0)
	{
		pack_info.PHONE_USB=0;		
	}	
	*/
	set_pack_status(&pack_info);
	if(pack_info.USB==1)
	{
		delayms(3000);
		BSP_SMB_USBIN_Status(&usb_type);
		usb_charging(usb_type);
	}	
	
	/* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Ensure that MSI is wake-up system clock */ 
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
	
	xQueue_pack_info = xQueueCreate( 5, sizeof( PACK_INFO ) );  
		
	xQueue_uart1_tx = xQueueCreate( 5, sizeof( UART1_TX_QUEUE ) );  
	
	//xQueue_uart1_rx = xQueueCreate( 3, sizeof( UART1_RX_QUEUE ) );

	xQueue_uart1_rx_handle = xQueueCreate( 5,  sizeof(RxBuffer_U1));
	
	xCountingSemaphore_smb = xSemaphoreCreateCounting( 10, 0 );
		
	xCountingSemaphore_attach= xSemaphoreCreateCounting( 10, 0 );
	
	xMutex_i2c = xSemaphoreCreateMutex();
	
	xSemaphore_uart1_int = xSemaphoreCreateBinary( );
	if(xSemaphore_uart1_int==NULL)
			printf("xSemaphore uart1 create fail\n\r");
	
	xSemaphore_uart2_int = xSemaphoreCreateBinary( );
	if(xSemaphore_uart2_int==NULL)
			printf("xSemaphore uart2 create fail\n\r");
	
	xSemaphore_uart3_int = xSemaphoreCreateBinary( );
	if(xSemaphore_uart3_int==NULL)
			printf("xSemaphore uart3 create fail\n\r");
	
	xSemaphore_wwdg = xSemaphoreCreateBinary( );
	if(xSemaphore_wwdg==NULL)
			printf("xSemaphore wwdg create fail\n\r");
	
	xCountingSemaphore_key = xSemaphoreCreateCounting( 10, 0 );
	if(xCountingSemaphore_key==NULL)
			printf("xCountingSemaphore key create fail\n\r");
	
	xCreatedEventGroup=xEventGroupCreate();
	if(xCreatedEventGroup==NULL)
			printf("xCreatedEventGroup create fail\n\r");		
	
	xTaskCreate((TaskFunction_t)UART2_task1,					"uart2_task1",		300,	NULL,2,	UART2_handle );
	xTaskCreate((TaskFunction_t)ADC_task2,						"adc_task2",			300,	NULL,2,	ADC_handle );
	xTaskCreate((TaskFunction_t)CHG_task3,						"chg_task3",			300,	NULL,2,	CHARGING_handle );
	xTaskCreate((TaskFunction_t)SMB_task4,						"smb_task4",			300,	NULL,3,	SMB_handle );
	xTaskCreate((TaskFunction_t)LED_task5,						"led_task5",			300,	NULL,2,	LED_handle );
	xTaskCreate((TaskFunction_t)UART1_RX_task6,				"uart1_rx_task6",	1000,	NULL,2,	UART1_RX_handle );
	xTaskCreate((TaskFunction_t)UART1_TX_task7,				"uart1_tx_task7",	300,	NULL,2,	UART1_TX_handle );
	xTaskCreate((TaskFunction_t)UART1_RX_HANDLE_task8,"rx_handle_task8",1000,	NULL,3,	UART1_RX_handle_pkt );
	xTaskCreate((TaskFunction_t)ATTACH_task9,					"attach_task9",		300,	NULL,2,	ATTACH_handle );
	xTaskCreate((TaskFunction_t)WDG_task10,						"wwdg_task10",		300,	NULL,4,	WDG_handle );
  
	/* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
	
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  static portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE;
	
	if(UartHandle->Instance==BSP_UART2)
	{
		if((xSemaphoreGiveFromISR( xSemaphore_uart2_int, &xHigherPriorityTaskWoken))!=pdPASS)
			printf("UART2 IRQ error\n\r");	
	}
	
	if(UartHandle->Instance==BSP_UART1)
	{
		if((xSemaphoreGiveFromISR( xSemaphore_uart1_int, &xHigherPriorityTaskWoken))!=pdPASS)
			printf("UART1 IRQ error\n\r");	
	}
	
	if(UartHandle->Instance==BSP_UART3)
	{
		if((xSemaphoreGiveFromISR( xSemaphore_uart3_int, &xHigherPriorityTaskWoken))!=pdPASS)
			printf("UART3 IRQ error\n\r");	
	}
}

/**
  * @brief WWDG callbacks
  * @param 
  * @retval None
  */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
  static portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE;
	
	/* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdg);
		
	
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  portBASE_TYPE xStatus;	
	
	static portBASE_TYPE *xHigherPriorityTaskWoken=pdFALSE;
	static portBASE_TYPE xHigherPriorityTaskWoken_smb=pdFALSE;
  static portBASE_TYPE xHigherPriorityTaskWoken_key=pdFALSE;
	
	if (GPIO_Pin == SMB_CC_STS_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == SMB_STAT_PIN)
  {
    if(HAL_GPIO_ReadPin(SMB_STAT_PIN_GPIO_PORT,SMB_STAT_PIN)==0)
		{
			if((xSemaphoreGiveFromISR( xCountingSemaphore_smb, &xHigherPriorityTaskWoken_smb))!=pdPASS)
				printf("SMB STATE PIN IRQ error\n\r");	
		}
  }

  else if (GPIO_Pin == SMB_SYS_PIN)
  {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == BOOST_OCP_INT_PIN)
  {
    printf("BAT OCP PIN IRQ\n\r");
		HAL_GPIO_WritePin(BOOST_ENABLE_PIN_GPIO_PORT, 	BOOST_ENABLE_PIN,		GPIO_PIN_SET);
  }

  else if (GPIO_Pin == KEY_PIN)
  {
    if((xSemaphoreGiveFromISR( xCountingSemaphore_key, &xHigherPriorityTaskWoken_key))!=pdPASS)
			printf("KEY IRQ error\n\r");
  }

  else if (GPIO_Pin == PHONE_ATTACHED_PIN)
  {
    if((xSemaphoreGiveFromISR( xCountingSemaphore_attach, &xHigherPriorityTaskWoken_key))!=pdPASS)
			printf("Attach IRQ error\n\r");		
  }

  else if (GPIO_Pin == FG_INT_PIN)
  {
    printf("Pack low battery!!\n\r");
		BSP_CW_Release_Alrt_Pin();
		pack_info.PACK_SOC=BSP_CW_Get_Capacity(pack_info.USB);
		xStatus=xQueueSendToBackFromISR( xQueue_pack_info, &pack_info, xHigherPriorityTaskWoken );
		if( xStatus != pdPASS )
		{
			printf("Send to queue error!\n\r");	
		}	
  }
  
}

/**
  * @brief  UART2 thread 1
  * @param  thread not used
  * @retval None
  */
static void UART2_task1(void const *argument)
{
	(void) argument;
  uint8_t i;
	char * s;	
	uint16_t RxData1=0;
	uint16_t RxData2=0;
	
	for(;;)
	{			
			SET_BIT(Uart2Handle.Instance->CR2, USART_CR2_RTOEN);	//receiver timeout enable
			SET_BIT(Uart2Handle.Instance->CR1, USART_CR1_RTOIE);	//receiver timeout interrupt enable
			Uart2Handle.Instance->RTOR=0x00000016;								//22x bit duration
			
			for(i=0;i<RXBUFFERSIZE;i++)
				*(RxBuffer+i)=0;
			if(HAL_UART_Receive_IT(&Uart2Handle, RxBuffer, RXBUFFERSIZE) != HAL_OK)
				 printf("uart int error\n\r");
			while(xSemaphoreTake( xSemaphore_uart2_int, 3000 )!=pdPASS)		//
			{
				xEventGroupSetBits(xCreatedEventGroup, TASK1_BIT);
			}
			DecodeReception(&RxData1,&RxData2);		  
      s = PbCommand;
      
      if (strcmp(s, "pbfw") == 0){pbfw();}
      else if (strcmp(s, "pbsoc") == 0) {pbsoc();}
      else if (strcmp(s, "pbphsoc") == 0) {pbphsoc(RxData1);}
      else if (strcmp(s, "pbusb") == 0) {pbusb(RxData1);}
      else if (strcmp(s, "pbwrsmb") == 0) {pbwr(RxData1,RxData2);}
      else if (strcmp(s, "pbrdsmb") == 0) {pbrd(RxData1);}
			else if (strcmp(s, "help") == 0) {pbhelp();}
			else if (strcmp(s, "pbadc") == 0) {pbadc();}
			else if (strcmp(s, "pbwrcw") == 0) {pbwrcw(RxData1,RxData2);}
      else if (strcmp(s, "pbrdcw") == 0) {pbrdcw(RxData1);}
			else if (strcmp(s, "pbchgen") == 0) {pbchgen(RxData1);}
			else if (strcmp(s, "pbpmuxena") == 0) {pbpmuxena(RxData1);}
			else if (strcmp(s, "pbpmuxenb") == 0) {pbpmuxenb(RxData1);}
			else if (strcmp(s, "pbboosten") == 0) {pbboosten(RxData1);}
			else if (strcmp(s, "pbboost9ven") == 0) {pbboost9ven(RxData1);}
			else if (strcmp(s, "pbsetpath") == 0) {set_pack_path(RxData1);}
      else {
         printf("Invalid command.. Please enter again\n\r");
         
         /* Reset received test number array */
         memset(RxCommand, 0, RXCOMMANDSIZE);
       }
    }	
}

/**
  * @brief  ADC thread 2
  * @param  thread not used
  * @retval None
  */
static void ADC_task2(void const *argument)
{
	uint32_t vrefint_data,vrefint_cal;
	vrefint_cal=(uint32_t)(*VREFINT_CAL_ADDR);		
	
	uint8_t temp;		
	
	for(;;)
	{
		vrefint_data=aADCxConvertedData[3];
		vdda=(3000*vrefint_cal)/vrefint_data;	

		usb_in_voltage=(vdda*aADCxConvertedData[0]*11)/4096;
		ext_pwr_voltage=(vdda*aADCxConvertedData[1]*11)/4096;
		acc_id_voltage=(vdda*aADCxConvertedData[2])/4096;
				
		xSemaphoreTake( xMutex_i2c, portMAX_DELAY );
		///////////////////////////////////////////////
		//take the priority of I2C access
		BSP_SMB_TADC_Start();
		vTaskDelay(5);
		BSP_SMB_TADC(&smb_tadc);
		pack_info.PACK_SOC=BSP_CW_Get_Capacity(pack_info.USB);
		cw_vbat=BSP_CW_GET_Vol();
		temp=BSP_SMB_USB_Detect();
		///////////////////////////////////////////////
		xSemaphoreGive( xMutex_i2c);
		
		if(temp!=(uint8_t)pack_info.USB)
		{
			printf("USB state changed!\n\r");
			xSemaphoreGive( xCountingSemaphore_smb);			
		}
		vTaskDelay(500);	
		
		xEventGroupSetBits(xCreatedEventGroup, TASK2_BIT);		
	}
}

/**
  * @brief  Charging path management thread 3
  * @param  thread not used
  * @retval None
  */
static void CHG_task3(void const *argument)
{
	PACK_INFO rptr;	
	portBASE_TYPE xStatus;
	uint8_t i;
	PACK_STATUS last_status;
	
	for(;;)
	{
		last_status=pack_status;
		xStatus=xQueueReceive( xQueue_pack_info, &rptr, 3000 );
		if (xStatus == pdPASS) 
		{
			set_pack_status(&rptr);
			i=0;
			//pack_status=STATUS_STANDBY;
		}
		else
		{
			if((pack_status==STATUS_STANDBY)&&(last_status==STATUS_STANDBY))
			{
				i++;
				if(i>3)
				{
					pack_status=STATUS_SLEEP;
					i=0;
				}
			}
		}
		xEventGroupSetBits(xCreatedEventGroup, TASK3_BIT);
	}
	
}

/**
  * @brief  SMB charging thread 4
  * @param  thread not used
  * @retval None
  */
static void SMB_task4(void const *argument)
{	
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;		
	
	UART1_TX_QUEUE tx_queue;
	
	memset(&tx_queue, 0, TXBUFFERSIZE_U1+1);	
	
	for(;;)
	{
		while(xSemaphoreTake( xCountingSemaphore_smb, 3000 )!=pdPASS)
			xEventGroupSetBits(xCreatedEventGroup, TASK4_BIT);
		
		xEventGroupSetBits(xCreatedEventGroup, TASK4_BIT);
		xSemaphoreTake( xMutex_i2c, portMAX_DELAY );
		BSP_SMB_INT_Type(&in_state);			
		
		//USB charging
		if(in_state.USBIN==1)
		{
			//BSP_SMB_USBIN_Exit_Suspend();
			pack_info.USB=1;	
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
			if( xStatus != pdPASS )
			{
				printf("Send to queue error!\n\r");	
			}	
			
			vTaskDelay(3000);
			BSP_SMB_USBIN_Status(&usb_type);
			usb_charging_task(usb_type);
			uart1_tx_charging_data_handle(&usb_type,&pack_info,&tx_queue);
			xStatus=xQueueSendToBack( xQueue_uart1_tx, &tx_queue, xTicksToWait );
			if( xStatus != pdPASS )
			{
				printf("Send to uart1 tx queue error!\n\r");	
			}		
		}
		else if(in_state.USBIN==0)
		{
			pack_info.USB=0;
			//BSP_SMB_USBIN_Suspend();
			uart1_tx_charging_data_handle(&usb_type,&pack_info,&tx_queue);
			xStatus=xQueueSendToBack( xQueue_uart1_tx, &tx_queue, xTicksToWait );
			if( xStatus != pdPASS )
			{
				printf("Send to uart1 tx queue error!\n\r");	
			}	
		}
		
		//DC_IN charging
		if(in_state.DCIN==0)
		{
			pack_info.PHONE_USB=0;			
			//BSP_SMB_DCIN_Suspend();
		}	
		xSemaphoreGive( xMutex_i2c);
		
		xSemaphoreGive( xCountingSemaphore_key);	
		
		
	}			
}

/**
  * @brief  LED and key handlling thread 5
  * @param  thread not used
  * @retval None
  */
static void LED_task5(void const *argument)
{
	portBASE_TYPE xStatus;
	uint8_t key;
	uint16_t xTicksToWait=500 / portTICK_RATE_MS;	
	uint8_t pcWriteBuffer[500];
	
	led_flash();
	//led_of_soc(pack_info.PACK_SOC,0);
	for(;;)
	{
		xStatus=xSemaphoreTake( xCountingSemaphore_key, 3000 );	
		if(xStatus==pdPASS)
		{			
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, 10 );
			if( xStatus != pdPASS )
			{
				printf("Send to queue error!\n\r");	
			}	
				
			key=HAL_GPIO_ReadPin(KEY_PIN_GPIO_PORT,KEY_PIN);
			led_of_soc(pack_info.PACK_SOC,key);
			if(key==0)
			{		
				
				printf("taskname		status	priority	freestack	num\r\n");
				vTaskList((char *)&pcWriteBuffer);
				printf("%s\r\n", pcWriteBuffer);
				while(HAL_GPIO_ReadPin(KEY_PIN_GPIO_PORT,KEY_PIN)==0)
				{
					vTaskDelay(100);
				}
			}
		}
		xEventGroupSetBits(xCreatedEventGroup, TASK5_BIT);
	}			
}

/**
  * @brief  UART1 rx int thread 6
  * @param  thread not used
  * @retval None
  */
static void UART1_RX_task6(void const *argument)
{
	(void) argument;
  uint8_t i;
	
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;		
	
	//portBASE_TYPE xStatus_queue;
	
	SET_BIT(Uart1Handle.Instance->CR2, USART_CR2_RTOEN);	//receiver timeout enable
	SET_BIT(Uart1Handle.Instance->CR1, USART_CR1_RTOIE);	//receiver timeout interrupt enable
	Uart1Handle.Instance->RTOR=0x0000050;									//80x bit duration
	
	for(;;)
	{			
		memset(RxBuffer_U1, 0, RXBUFFERSIZE_U1);		
		if(HAL_UART_Receive_IT(&Uart1Handle, RxBuffer_U1, RXBUFFERSIZE_U1) == HAL_OK)
		{	
			while(xSemaphoreTake( xSemaphore_uart1_int, 3000 )!=pdPASS)
				xEventGroupSetBits(xCreatedEventGroup, TASK6_BIT);				
			
			xStatus=xQueueSendToBack( xQueue_uart1_rx_handle, &RxBuffer_U1, 20 );
			/*
			for(i=0;i<RXBUFFERSIZE_U1;i++)
				printf("%x",*(RxBuffer_U1+i));
			printf("\n\r");
			*/
			printf("task6 rx data\n\r");
			
			if( xStatus != pdPASS )
			{
				printf("UART1 rx task Send to uart1 rx handel queue error!\n\r");	
			}	
		}
		else
		 printf("uart1 int error\n\r");
	}
}

/**
  * @brief  UART1 tx thread 7
  * @param  thread not used
  * @retval None
  */
static void UART1_TX_task7(void const *argument)
{
	(void) argument;
  uint8_t length;
	
	//portBASE_TYPE xStatus;
	//uint16_t xTicksToWait=100 / portTICK_RATE_MS;			
	
	UART1_TX_QUEUE tx_queue;		
	
	for(;;)
	{			
		memset(&tx_queue, 0, TXBUFFERSIZE_U1+1);
		while(xQueueReceive(xQueue_uart1_tx, &tx_queue, 3000 )!=pdPASS)
			xEventGroupSetBits(xCreatedEventGroup, TASK7_BIT);
		
		length=tx_queue.TX_BUF[2]+3;
		HAL_UART_Transmit(&Uart1Handle,tx_queue.TX_BUF,length,100);
			
					
	}
}

/**
  * @brief  UART1 rx buffer command parsing thread 8
  * @param  thread not used
  * @retval None
  */
static void UART1_RX_HANDLE_task8(void const *argument)
{
	(void) argument;
  uint8_t i,ack_num,length;
	
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=1000 / portTICK_RATE_MS;		
	
	PHONE_INFO phone_info;
	phone_info.EXTPWR_VOLTAGE=0;
	phone_info.PHONE_SOC=0;	
	
	uint8_t 				packet[RXBUFFERSIZE_U1]={0};
	for(;;)
	{			
		while(xQueueReceive(xQueue_uart1_rx_handle, &packet, 3000 )!=pdPASS)
			xEventGroupSetBits(xCreatedEventGroup, TASK8_BIT);
		/*
		for(i=0;i<RXBUFFERSIZE_U1;i++)
			printf("0x%x,",*(packet+i));
		*/
		printf("task8 rx handle\n\r");
		
		ack_num=0;
		uart1_rx_ack_handle(packet,&ack_num, &tx_queue[0],&phone_info);
		
		if(ack_num!=0)
		{
			
			for(i=0;i<ack_num;i++)
			{
				length=tx_queue[i].TX_BUF[2]+3;
				HAL_UART_Transmit(&Uart1Handle,tx_queue[i].TX_BUF,length,100);
				
				/*tx_queue[i].ACK_PACK=PACK_ACK;
				xStatus=xQueueSendToBack( xQueue_uart1_tx, &tx_queue[i], xTicksToWait );
				//printf("%d txqueue:%x,%x\n\r",i,tx_queue[i].TX_BUF[3],tx_queue[i].TX_BUF[4]);
				if( xStatus != pdPASS )
				{
					printf("UART1 rx task8 Send to uart1 tx queue error!\n\r");	
				}	
				*/
			}
			
			pack_info.PHONE_SOC=phone_info.PHONE_SOC;
			pack_info.PHONE_USB_VOLTAGE=phone_info.EXTPWR_VOLTAGE;
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, 5 );
			if( xStatus != pdPASS )
			{
				printf("UART1 rx task8 Send to pack info queue error!\n\r");	
			}								
		}		
	}
}

/**
  * @brief  Attach int handlling thread 9
  * @param  thread not used
  * @retval None
  */
static void ATTACH_task9(void const *argument)
{	
	portBASE_TYPE xStatus;	
	uint16_t xTicksToWait=2 / portTICK_RATE_MS;		
	uint8_t attach;
	for(;;)
	{
		while(xSemaphoreTake( xCountingSemaphore_attach, 3000 )!=pdPASS)
			xEventGroupSetBits(xCreatedEventGroup, TASK9_BIT);

		//printf("task9 attach irq\n\r");	
		if(HAL_GPIO_ReadPin(PHONE_ATTACHED_PIN_GPIO_PORT,PHONE_ATTACHED_PIN))
		{
			attach=0;
			pack_info.PHONE_SOC=0;
			pack_info.PHONE_USB=0;
			pack_info.PHONE_USB_VOLTAGE=0;
		}
		else
			attach=1;
		
		if(attach!=pack_info.ATTACH)
		{
			pack_info.ATTACH=attach;
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );		
			if( xStatus != pdPASS )
			{
				printf("task 9 Send to pack info queue error!\n\r");	
			}		
		}
				
	}
}

/**
  * @brief  IWDG watchdog thread 10
  * @param  thread not used
  * @retval None
  */
static void WDG_task10(void const *argument)
{	
	EventBits_t uxBits;
	vTaskDelay(200);
	BSP_WWDG_Init();
	wdg_count=0;
	for(;;)
	{
		//xSemaphoreTake( xCountingSemaphore_attach, portMAX_DELAY );
		while(wdg_count<500)
		{
			vTaskDelay(15);
			HAL_WWDG_Refresh(&WwdgHandle);	
			wdg_count++;			
		}
		uxBits=xEventGroupWaitBits(xCreatedEventGroup, TASK_BIT_ALL, pdTRUE, pdTRUE, 10);
		if((uxBits&TASK_BIT_ALL)==TASK_BIT_ALL)
		{
			wdg_count=0;					
		}
		else
			printf("uxbits=0x%x\n\r",uxBits);
	}
}


void uart1_tx_charging_data_handle(USB_TYPE *usb_type, PACK_INFO *pk_info, UART1_TX_QUEUE *tx_queue)
{
	//uint8_t length,cmd_type, cmd;	
	UART1_CHARGING charging_packet;
	
	//uint16_t phone_voltage, phone_current;
	//uint8_t phone_soc,num ,size;
	//uint8_t gpio;
	uint16_t crc;
	
	charging_packet.SOF_HIGH=0x55;
	charging_packet.SOF_LOW=0xaa;
	charging_packet.LENGTH=sizeof(UART1_CHARGING)-3;
	charging_packet.CMD_TYPE=CMD_CHARGING;
	charging_packet.CMD=0x01;
	
	if(pk_info->USB==1)
	{
		charging_packet.VOLTAGE_HIGH=(uint8_t)((usb_in_voltage&0x0ffff)>>8);
		charging_packet.VOLTAGE_LOW=(uint8_t)(usb_in_voltage&0x0ff);
		if((usb_type->CHARGER==QC2_CHARGER)||(usb_type->CHARGER==QC3_CHARGER))
		{
			charging_packet.CURRENT_HIGH=(uint8_t)((2000&0x0ffff)>>8);
			charging_packet.CURRENT_LOW=(uint8_t)(2000&0x0ff);
		}
		else
		{
			charging_packet.CURRENT_HIGH=(uint8_t)((500&0x0ffff)>>8);
			charging_packet.CURRENT_LOW=(uint8_t)(500&0x0ff);
		}
	}
	else
	{
		charging_packet.VOLTAGE_HIGH=0;
		charging_packet.VOLTAGE_LOW=0;	
		charging_packet.CURRENT_HIGH=0;
		charging_packet.CURRENT_LOW=0;
	}	
	charging_packet.ACC=acc_type;
	charging_packet.STATUS=pack_status;
	crc=crc16(&charging_packet.SOF_HIGH,sizeof(charging_packet)-2);
	charging_packet.CRC_HIGH=(uint8_t )(crc>>8);
	charging_packet.CRC_LOW=(uint8_t )(crc&0xFF);
	tx_queue->ACK_PACK=PACK_NO_ACK;
	memcpy(tx_queue->TX_BUF,&charging_packet,sizeof(charging_packet));				
}


void uart1_rx_no_ack_handle(uint8_t *rxdata, PHONE_INFO *info)
{
	uint8_t length,cmd_type;
	//uint8_t cmd,num ,size;		
	uint16_t phone_voltage, phone_current;
	uint8_t phone_soc ;
	uint8_t gpio;
	uint16_t rx_crc,crc;
	
	if((*rxdata==0xaa)&&(*(rxdata+1)==0x55))
	{
		length=*(rxdata+2);
		cmd_type=*(rxdata+3);
		//cmd=*(rxdata+4);		
		
		if(cmd_type==CMD_CHARGING)//charging
		{
			phone_voltage=*(rxdata+5);
			phone_voltage=(phone_voltage<<8)+*(rxdata+6);			
			
			phone_current=*(rxdata+7);
			phone_current=(phone_current<<8)+*(rxdata+8);
			phone_soc=*(rxdata+9);			
			
			gpio=*(rxdata+10);
			rx_crc=*(rxdata+11);
			rx_crc=(rx_crc<<8)+*(rxdata+12);
			
			crc=crc16(rxdata,length+1);
			if(rx_crc==crc)	//crc ok
			{
				if((phone_voltage>=4800)&&(phone_voltage<=12000))
				{
					info->EXTPWR_VOLTAGE=phone_voltage;
					info->PHONE_SOC=phone_soc;
				}	
				else
				{
					info->EXTPWR_VOLTAGE=0;
					info->PHONE_SOC=0;
				}	
				
				if(gpio&0x01)
					HAL_GPIO_WritePin(GPIO1_F_PIN_GPIO_PORT, 		GPIO1_F_PIN,		GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIO1_F_PIN_GPIO_PORT, 		GPIO1_F_PIN,		GPIO_PIN_RESET);
				
				if(gpio&0x02)
					HAL_GPIO_WritePin(GPIO2_F_PIN_GPIO_PORT, 		GPIO2_F_PIN,		GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIO2_F_PIN_GPIO_PORT, 		GPIO2_F_PIN,		GPIO_PIN_RESET);
			}//crc ok
			else	//crc error
			{
				
			}//crc error
		}// cmd charging
		else if(cmd_type==CMD_UART_TRANSFER)//uart transfer
		{		
			//num=*(rxdata+5);
			//size=*(rxdata+6);
			rx_crc=*(rxdata+7);
			rx_crc=(rx_crc<<8)+*(rxdata+8);
			
			crc=crc16(rxdata,length+1);
			if(rx_crc!=crc)	//crc error
			{
				printf("UART1 RX CRC error\n\r");
			}//crc error
					
		}//uart transfer		
	}//	SOF OK
	else
		printf("UART1 RX SOF error\n\r");
}


void uart1_rx_ack_handle(uint8_t *uart_rxdata, uint8_t *queue_num, UART1_TX_QUEUE *tx_ack_queue, PHONE_INFO *info)
{
	uint8_t length,cmd_type, cmd;	
	UART1_CHARGING_ACK charging_packet;
	UART1_TRANSFER_INITIAL transfer_packet_initial;
	//UART1_TRANSFER_DATA transfer_packet_data;
	UART1_FIRMWARE_ACK fw_packet;
	uint8_t *rxdata;
	
	uint16_t phone_voltage, phone_current;
	uint8_t phone_soc,num ,size;
	uint8_t gpio;
	uint16_t rx_crc,crc;
	uint8_t i=0,j=0;
	*queue_num=0;
	
	while(i<RXBUFFERSIZE_U1)
	{
		if((*(uart_rxdata+i)==0xaa)&&(*(uart_rxdata+i+1)==0x55))
		{
			rxdata=uart_rxdata+i;
			(*queue_num)++;
			if((*rxdata==0xaa)&&(*(rxdata+1)==0x55))
			{
				length=*(rxdata+2);
				i=i+length+3;
				cmd_type=*(rxdata+3);
				cmd=*(rxdata+4);		
				
				if(cmd_type==CMD_CHARGING_ACK)//charging
				{
					phone_voltage=*(rxdata+5);
					phone_voltage=(phone_voltage<<8)+*(rxdata+6);			
					
					phone_current=*(rxdata+7);
					phone_current=(phone_current<<8)+*(rxdata+8);
					phone_soc=*(rxdata+9);
					info->PHONE_SOC=phone_soc;
					//info->EXTPWR_VOLTAGE=phone_voltage;
					
					gpio=*(rxdata+10);
					rx_crc=*(rxdata+11);
					rx_crc=(rx_crc<<8)+*(rxdata+12);
					
					crc=crc16(rxdata,length+1);
					if(rx_crc==crc)	//crc ok
					{
						charging_packet.SOF_HIGH=0x55;
						charging_packet.SOF_LOW=0xaa;
						charging_packet.LENGTH=12;
						charging_packet.CMD_TYPE=CMD_CHARGING;
						charging_packet.CMD=0x01;
						if((phone_voltage>=4800)&&(phone_voltage<=12000))
						{
							charging_packet.VOLTAGE_HIGH=*(rxdata+5);
							charging_packet.VOLTAGE_LOW=*(rxdata+6);
							charging_packet.CURRENT_HIGH=*(rxdata+7);
							charging_packet.CURRENT_LOW=*(rxdata+8);
							info->EXTPWR_VOLTAGE=phone_voltage;
							printf("phone USB voltage:%dmV\n\r",phone_voltage);
						}	
						else
						{
							charging_packet.VOLTAGE_HIGH=0;
							charging_packet.VOLTAGE_LOW=0;
							charging_packet.CURRENT_HIGH=0;
							charging_packet.CURRENT_LOW=0;
							info->EXTPWR_VOLTAGE=0;
						}	
						
						if(gpio&0x01)
							HAL_GPIO_WritePin(GPIO1_F_PIN_GPIO_PORT, 		GPIO1_F_PIN,		GPIO_PIN_SET);
						else
							HAL_GPIO_WritePin(GPIO1_F_PIN_GPIO_PORT, 		GPIO1_F_PIN,		GPIO_PIN_RESET);
						
						if(gpio&0x02)
							HAL_GPIO_WritePin(GPIO2_F_PIN_GPIO_PORT, 		GPIO2_F_PIN,		GPIO_PIN_SET);
						else
							HAL_GPIO_WritePin(GPIO2_F_PIN_GPIO_PORT, 		GPIO2_F_PIN,		GPIO_PIN_RESET);
						charging_packet.SOC=pack_info.PACK_SOC;
						charging_packet.ACC=acc_type;
						charging_packet.CHARGE_STATUS=pack_status;
						charging_packet.STATUS=0x90;
						crc=crc16(&charging_packet.SOF_HIGH,sizeof(charging_packet)-2);
						charging_packet.CRC_HIGH=(uint8_t )(crc>>8);
						charging_packet.CRC_LOW=(uint8_t )(crc&0xFF);
						j=*queue_num-1;
						memcpy((tx_ack_queue+j)->TX_BUF,&charging_packet,sizeof(charging_packet));				
					}//crc ok
					else	//crc error
					{
						printf("crc error\n\r");
					}//crc error
				}// cmd charging
				else if(cmd_type==CMD_UART_TRANSFER_ACK)//uart transfer
				{
					if(cmd==0x00)		//initial packet
					{
						num=*(rxdata+5);
						size=*(rxdata+6);
						
						uart1_packet_num=num;
						uart1_packet_size=size;
						
						rx_crc=*(rxdata+7);
						rx_crc=(rx_crc<<8)+*(rxdata+8);
						
					}
					else
					{
						rx_crc=*(rxdata+length+1);
						rx_crc=(rx_crc<<8)+*(rxdata+length+2);
						
					}
					crc=crc16(rxdata,length+1);
					if(rx_crc==crc)	//crc ok
					{
						transfer_packet_initial.SOF_HIGH=0x55;
						transfer_packet_initial.SOF_LOW=0xaa;
						transfer_packet_initial.LENGTH=6;
						transfer_packet_initial.CMD_TYPE=CMD_UART_TRANSFER;
						transfer_packet_initial.CMD=cmd;
						transfer_packet_initial.NUM=uart1_packet_num;
						transfer_packet_initial.SIZE=uart1_packet_size;
						
						crc=crc16(&transfer_packet_initial.SOF_HIGH,sizeof(transfer_packet_initial)-2);
						transfer_packet_initial.CRC_HIGH=(uint8_t )(crc>>8);
						transfer_packet_initial.CRC_LOW=(uint8_t )(crc&0xFF);
						j=*queue_num-1;
						memcpy((tx_ack_queue+j)->TX_BUF,&transfer_packet_initial,sizeof(transfer_packet_initial));	
						
										
					}//initial packet
					else	//crc error
					{
						printf("uart1 rx UART transfer crc error\n\r");
					}//crc error					
				}//uart transfer
				else if(cmd_type==CMD_FIRMWARE_ACK)//fimware version
				{
					rx_crc=*(rxdata+5);
					rx_crc=(rx_crc<<8)+*(rxdata+6);
					crc=crc16(rxdata,length+1);
					if(rx_crc==crc)				
					{
						fw_packet.SOF_HIGH=0x55;
						fw_packet.SOF_LOW=0xaa;
						fw_packet.LENGTH=7;
						fw_packet.CMD_TYPE=CMD_FIRMWARE;
						fw_packet.CMD=0x00;
						fw_packet.MVERSION=__PACK_BSP_VERSION_MAIN;
						fw_packet.SVERSION=__PACK_BSP_VERSION_SUB;
						fw_packet.STATUS=0x90;
						crc=crc16(&fw_packet.SOF_HIGH,sizeof(fw_packet)-2);
						fw_packet.CRC_HIGH=(uint8_t )(crc>>8);
						fw_packet.CRC_LOW=(uint8_t )(crc&0xFF);
						j=*queue_num-1;
						memcpy((tx_ack_queue+j)->TX_BUF,&fw_packet,sizeof(fw_packet));
					}	
					else	//crc error
					{
						printf("uart1 rx firmare crc error\n\r");
					}//crc error					
				}	
			}
			else
				printf("UART1 RX SOF error\n\r");
		}
		else
			i++;
	}//while
}

uint16_t crc16(uint8_t *data, uint8_t data_len)
{
	uint16_t crc_out=0x0000;
	
	crc_out=	HAL_CRC_Calculate(&CrcHandle, (uint32_t *)data, data_len);
	return crc_out;
}

/*
uint16_t crc16(uint8_t *data, uint8_t data_len)
{
	uint16_t crc_in=0x0000;
	uint16_t crc_poly=0x1021;
	uint8_t c_char=0;
	uint8_t i;
	while(data_len--)
	{
		c_char=*(data++);
		invert_uint8(&c_char,&c_char);
		crc_in ^=(c_char<<8);
		for(i=0;i<8;i++)
		{
			if(crc_in&0x8000)
				crc_in=(crc_in<<1)^crc_poly;
			else
				crc_in=(crc_in<<1);			
		}		
	}	
	invert_uint16(&crc_in,&crc_in);
	return crc_in;
}
*/
void invert_uint8(uint8_t *dbuff, uint8_t *srcbuff)
{
	uint8_t i;
	uint8_t temp=0;
	for(i=0;i<8;i++)
	{
		if(*srcbuff&(1<<i))
			temp |=1<<(7-i);
	}
	*dbuff=temp;	
}

void invert_uint16(uint16_t *dbuff, uint16_t *srcbuff)
{
	uint8_t i;
	uint16_t temp=0;
	for(i=0;i<16;i++)
	{
		if(*srcbuff&(1<<i))
			temp |=1<<(15-i);
	}
	*dbuff=temp;	
}

void usb_charging(USB_TYPE usb_type)
{	
	uint8_t pulse,i;
	int32_t max_current,current;
	if(usb_type.CHARGER==QC3_CHARGER)
	{
		if(pack_status==STATUS_USB2PACK)
		{
			i=0;
			max_current=0;
			BSP_SMB_QC2_Force_5V();
			delayms(400);
			while(i<20)
			{
				BSP_SMB_BAT_Current_Start();
				delayms(50);
				
				current=BSP_SMB_BAT_Current();
				printf("BAT current:%dmA\n\r",current);
				if(current>max_current)
				{
					max_current=current;
					pulse=i;
				}
				BSP_SMB_QC3_Single_Inc();
				i++;
			}
			if(pulse!=0)
			{
				i=20-pulse;
				printf("QC3 pulse=%d\n\r",pulse);
				while(i>0)
				{
					BSP_SMB_QC3_Single_Dec();
					delayms(20);
					i--;
				}			
			}
		}
		else if(pack_status==STATUS_USB2PHONE)
		{
			BSP_SMB_QC2_Force_5V();
			delayms(400);
			BSP_SMB_QC2_Force_9V();
			printf("USB to Phone set to 9V\n\r");
		}		
	}
	else if(usb_type.CHARGER==QC2_CHARGER)
	{
		BSP_SMB_QC2_Force_5V();
		delayms(100);
		BSP_SMB_QC2_Force_9V();
		printf("QC2 charger set to 9V\n\r");
	}				
}

void usb_charging_task(USB_TYPE usb_type)
{	
	uint8_t pulse,i;
	int32_t max_current,current;
	if(usb_type.CHARGER==QC3_CHARGER)
	{
		if(pack_status==STATUS_USB2PACK)
		{
			i=0;
			max_current=0;
			BSP_SMB_QC2_Force_5V();
			
			while(i<20)
			{
				BSP_SMB_BAT_Current_Start();
				vTaskDelay(50);			
				current=BSP_SMB_BAT_Current();
				printf("BAT current:%dmA\n\r",current);
				if(current>max_current)
				{
					max_current=current;
					pulse=i;
				}
				BSP_SMB_QC3_Single_Inc();
				//vTaskDelay(400);
				i++;
			}
			if(pulse!=0)
			{
				i=20-pulse;
				printf("QC3 pulse=%d\n\r",pulse);
				while(i>0)
				{
					BSP_SMB_QC3_Single_Dec();
					vTaskDelay(10);
					i--;
				}			
			}
		}
		else if(pack_status==STATUS_USB2PHONE)
		{
			BSP_SMB_QC2_Force_5V();
			vTaskDelay(100);	
			BSP_SMB_QC2_Force_9V();
			printf("USB to Phone set to 9V\n\r");
		}			
	}
	else if(usb_type.CHARGER==QC2_CHARGER)
	{
		BSP_SMB_QC2_Force_5V();
		vTaskDelay(10);	
		BSP_SMB_QC2_Force_9V();
		printf("QC2 charger set to 9V\n\r");
	}				
}

void led_flash(void)
{
	uint8_t i=0;
	while(i<3)
	{
		BSP_LED_On(LED1);
		BSP_LED_On(LED2);
		BSP_LED_On(LED3);
		BSP_LED_On(LED4);
		vTaskDelay(50);
		BSP_LED_Off(LED1);
		BSP_LED_Off(LED2);
		BSP_LED_Off(LED3);
		BSP_LED_Off(LED4);
		vTaskDelay(50);
		i++;
	}
}

void led_of_soc(uint8_t soc, uint8_t key)
{
	uint8_t i=5;
	if((pack_info.PHONE_USB==0)&&(pack_info.USB==0))
	{
		if(key==1)
		{
			if(soc>80)
			{
				BSP_LED_On(LED1);
				BSP_LED_On(LED2);
				BSP_LED_On(LED3);
				BSP_LED_On(LED4);
				vTaskDelay(2000);
				
			}
			else if((soc<=80)&&(soc>50))
			{
				BSP_LED_On(LED1);
				BSP_LED_On(LED2);
				BSP_LED_On(LED3);
				BSP_LED_Off(LED4);
				vTaskDelay(2000);
			}
			else if((soc<=50)&&(soc>25))
			{
				BSP_LED_On(LED1);
				BSP_LED_On(LED2);
				BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
				vTaskDelay(2000);
			}
			else if((soc<=25)&&(soc>5))
			{
				BSP_LED_On(LED1);
				BSP_LED_Off(LED2);
				BSP_LED_Off(LED3);
				BSP_LED_Off(LED4);
				vTaskDelay(2000);
			}
			else
			{
				while(i>0)
				{
					BSP_LED_Toggle(LED1);
					vTaskDelay(200);	
					i--;
				}			
			}
		}
		
		BSP_LED_Off(LED1);
		BSP_LED_Off(LED2);
		BSP_LED_Off(LED3);
		BSP_LED_Off(LED4);
		
	}
	else
	{
		if(soc>80)
		{
			BSP_LED_On(LED1);
			BSP_LED_On(LED2);
			BSP_LED_On(LED3);
			BSP_LED_On(LED4);			
		}
		else if((soc<=80)&&(soc>50))
		{
			BSP_LED_On(LED1);
			BSP_LED_On(LED2);
			BSP_LED_On(LED3);
			BSP_LED_Off(LED4);			
		}
		else if((soc<=50)&&(soc>25))
		{
			BSP_LED_On(LED1);
			BSP_LED_On(LED2);
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);			
		}
		else
		{
			BSP_LED_On(LED1);
			BSP_LED_Off(LED2);
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);			
		}		
	}
}

/**
  * @brief  Decode recpeption of RBuffer
  * @param  None
  * @retval None
  */
void DecodeReception(uint16_t *d1,uint16_t *d2)
{
	uint16_t i;
	char temp;
	uint8_t rxdatanumber=0; 
	uint8_t d1_hex=0;
	uint8_t d2_hex=0;
	uint16_t rxdata1,rxdata2;
	
	for(i=0;i<RXCOMMANDSIZE;i++)
	{	
		PbCommand[i]=0;
		RxCommand[i]=0;
	}
	rxdatanumber=0;
	rxdata1=0;
	rxdata2=0;
	
	for(i=0;RxBuffer[i]!=CR_ASCII_VALUE;i++)
	{
		temp=RxBuffer[i];
		
		if(temp != SPACE_ASCII_VALUE)
		{
			RxCommand[i]=temp;
		}
		else
		{
			rxdatanumber++;
			if(rxdatanumber==1)
			{
				strcpy(PbCommand,RxCommand);
			}
		}
		
		if(rxdatanumber>0)
		{
			if((temp>='0')&&(temp<='9'))
			{
				if(rxdatanumber==1)
				{
					if(d1_hex==1)
						rxdata1=(rxdata1<<4)+(temp-0x30);
					else
						rxdata1=(rxdata1*10)+(temp-0x30);
					
				}
				else if(rxdatanumber==2)
				{
					if(d2_hex==1)
						rxdata2=(rxdata2<<4)+(temp-0x30);
					else
						rxdata2=(rxdata2*10)+(temp-0x30);
				}	
			}
			else if(temp=='x')
			{
				if(rxdatanumber==1)
				{
					rxdata1=0;
					d1_hex=1;					
				}
				else if(rxdatanumber==2)
				{
					rxdata2=0;
					d2_hex=1;					
				}	
			}
			else if((temp>='a')&&(temp<='f'))
			{
				if(rxdatanumber==1)
				{
					rxdata1=(rxdata1<<4)+(temp-0x57);
				}				
				else if(rxdatanumber==2)
				{
					rxdata2=(rxdata2<<4)+(temp-0x57);
					
				}	
			}
		}
	}
	if(rxdatanumber==0)
			strcpy(PbCommand,RxCommand);
	
	printf(PbCommand);
	if(rxdatanumber==1)
	{
		if(d1_hex==1)
			printf(" 0x%x",rxdata1);
		else
			printf(" %d",rxdata1);
	}
	else if(rxdatanumber==2)
	{	
		if(d2_hex==1)
			printf(" 0x%x 0x%x",rxdata1,rxdata2);
		else
			printf(" %d %d",rxdata1,rxdata2);
	}
	printf("\n\r");
	
	*d1=rxdata1;
	*d2=rxdata2;
}

void pbhelp(void)
{
	printf("****************Power Bank Command*****************\n\r");
	printf("pbfw		--PB firmware version\n\r");
	printf("pbsoc		--PB battery soc\n\r");
	printf("pbphsoc		--Phone battery soc\n\r");
	printf("pbusb xx		--Phone USB voltage\n\r");
	printf("pbwrsmb addr data	--I2C write SMB registor\n\r");
	printf("pbrdsmb addr	--I2C read SMB registor\n\r");
	printf("pbwrcw addr data	--I2C write CW2015 registor\n\r");
	printf("pbrdcw addr	--I2C read CW2015 registor\n\r");
	printf("pbadc		--PB ADC info\n\r");
	printf("pbchgen	1/0	--PB charge path enable/disable\n\r");
	printf("pbpmuxena	1/0	--PB pmux1 cha enable/disable\n\r");
	printf("pbpmuxenb	1/0	--PB pmux1 chb enable/disable\n\r");
	printf("pbboosten	1/0	--PB 5V boost enable/disable\n\r");
	printf("pbboost9ven 1/0	--PB 9V boost enable/disable\n\r");
	printf("pbsetpath	0-5	--PB set charging path\n\r");
	printf("	-0	--path auto\n\r");
	printf("	-1	--path pack usb to pack\n\r");
	printf("	-2	--path pack usb to pack&phone\n\r");
	printf("	-3	--path pack usb to phone\n\r");
	printf("	-4	--path pack boost 9v to phone\n\r");
	printf("	-5	--path phone usb to pack\n\r");
	printf("help		--help\n\r");
	printf("***************************************************\n\r");
}

void pbpmuxena(uint8_t io)
{
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_SET);
	if(io)
		HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		GPIO_PIN_RESET);	
}

void pbpmuxenb(uint8_t io)
{
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_SET);
	if(io)
		HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		GPIO_PIN_RESET);	
}

void pbchgen(uint8_t io)
{
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_SET);
	if(io)
		HAL_GPIO_WritePin(CHG_PATH_EN_PIN_GPIO_PORT, 		CHG_PATH_EN_PIN,		GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CHG_PATH_EN_PIN_GPIO_PORT, 		CHG_PATH_EN_PIN,		GPIO_PIN_RESET);
}

void pbboosten(uint8_t en)
{
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		GPIO_PIN_RESET);
	if(en)
		HAL_GPIO_WritePin(BOOST_ENABLE_PIN_GPIO_PORT, 	BOOST_ENABLE_PIN,		GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(BOOST_ENABLE_PIN_GPIO_PORT, 	BOOST_ENABLE_PIN,		GPIO_PIN_RESET);	
}

void pbboost9ven(uint8_t en)
{
	HAL_GPIO_WritePin(PMUX1_MODE_PIN_GPIO_PORT, 		PMUX1_MODE_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CTRL_PIN_GPIO_PORT, 		PMUX1_CTRL_PIN, 		GPIO_PIN_SET);
	HAL_GPIO_WritePin(PMUX1_CHA_EN_PIN_GPIO_PORT, 	PMUX1_CHA_EN_PIN,		GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PMUX1_CHB_EN_PIN_GPIO_PORT, 	PMUX1_CHB_EN_PIN,		GPIO_PIN_RESET);
	if(en)
		HAL_GPIO_WritePin(BOOST_9V_EN_PIN_GPIO_PORT, 		BOOST_9V_EN_PIN,		GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(BOOST_9V_EN_PIN_GPIO_PORT, 		BOOST_9V_EN_PIN,		GPIO_PIN_RESET);	
}

void pbfw(void)
{
	printf("PACK firmware version:%d.%d\n\r",__PACK_BSP_VERSION_MAIN,__PACK_BSP_VERSION_SUB);
}

void pbadc(void)
{
	printf("\n\r");
	printf("******SMB TADC******\n\r");
	printf("CH1 THERM1 NTC:%dC\n\r",smb_tadc.CH1_THERM1_NTC);
	//printf("CH2 THERM2 NTC:%dC\n\r",smb_tadc.CH2_THERM2_NTC);
	printf("CH3 DIE TEMP:%dC\n\r",smb_tadc.CH3_DIE_TEMP);
	printf("CH4 BAT CURRENT:%dmA\n\r",smb_tadc.CH4_BAT_CURRENT);
	printf("CH5 BAT VOLTAGE:%dmV\n\r",smb_tadc.CH5_BAT_VOL);
	printf("CH6 VIN CURRENT:%dmA\n\r",smb_tadc.CH6_VIN_CURRENT);
	printf("CH7 VIN VOLTAGE:%dmV\n\r",smb_tadc.CH7_VIN_VOL);
	printf("********************\n\r");
	printf("USB IN VOLTAGE:%dmV\n\r",usb_in_voltage);
	printf("EXT PWR VOLTAGE:%dmV\n\r",ext_pwr_voltage);
	printf("ACC ID VOLTAGE:%dmV\n\r",acc_id_voltage);
	printf("VDDA VOLTAGE:%dmV\n\r",vdda);
	printf("PACK SOC:%d\n\r",pack_info.PACK_SOC);	
	printf("CW VBAT:%d\n\r",cw_vbat);
	printf("\n\r");
}

void pbsoc(void)
{	
	printf("PACK soc:%d\n\r",pack_info.PACK_SOC);	
}

void pbphsoc(uint8_t soc)
{	
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;
	pack_info.PHONE_SOC=soc;
	printf("PHONE soc:%d\n\r",pack_info.PHONE_SOC);
			
	xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
	if( xStatus != pdPASS )
	{
		printf("Send to queue error!\n\r");	
	}
}

void pbusb(uint16_t usb_vol)
{
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;
	pack_info.PHONE_USB=1;
	pack_info.PHONE_USB_VOLTAGE=usb_vol;
	if(pack_info.PHONE_USB_VOLTAGE)
		pack_info.PHONE_USB=1;
	else
		pack_info.PHONE_USB=0;
	printf("PHONE USB VOLTAGE:%dmV\n\r",pack_info.PHONE_USB_VOLTAGE);	
	xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
	if( xStatus != pdPASS )
	{
		printf("Send to queue error!\n\r");	
	}
}

void pbwr(uint16_t addr, uint16_t data)
{
	printf("Write reg_add16:0x%x,reg_data=0x%x\n\r",addr,data);
	if((BSP_I2C2_Write(SMB_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, data))!=HAL_OK)
		printf("SBM1381 setting error!\n\r");		
}

void pbrd(uint16_t addr)
{
	uint8_t data;
	data=BSP_I2C2_Read(SMB_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT);
	printf("Read reg_add16:0x%x,reg_data=0x%x\n\r",addr,data);	
}

void pbwrcw(uint16_t addr, uint16_t data)
{
	printf("Write CW2015 reg_add8:0x%x,reg_data=0x%x\n\r",addr,data);
	if((BSP_I2C2_Write(CW_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data))!=HAL_OK)
		printf("CW write error!\n\r");		
}

void pbrdcw(uint16_t addr)
{
	uint8_t data;
	data=BSP_I2C2_Read(CW_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT);
	printf("Read CW2015 reg_add8:0x%x,reg_data=0x%x\n\r",addr,data);	
}


PACK_STATUS get_pack_status(PACK_INFO *ptr)
{	
	PACK_STATUS status;
	
	if((ptr->ATTACH==0)&&	(ptr->USB==0))
		status=STATUS_STANDBY;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==0)&&(ptr->PACK_SOC<=3))		
		status=STATUS_STANDBY;
	else if((ptr->ATTACH==0)&&	(ptr->USB==1))
		status=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==1)&&(ptr->PHONE_USB_VOLTAGE==0))		
		status=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC>=90))
		status=STATUS_USB2BOTH;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC<90))
		status=STATUS_USB2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==0)&&(ptr->PACK_SOC>3))
		status=STATUS_BOOST2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==1)&&(ptr->PHONE_USB_VOLTAGE>=5000))
		status=STATUS_PHONE2PACK;
	
	return status;
}

void set_pack_path(uint8_t path)
{
	switch(path){
			case 0:
			{
				BSP_STANDBY();
				printf("standby\n\r");
				break;
			}
			case 1:
			{
				BSP_USB2PACK();
				printf("usb to pack\n\r");
				break;
			}
			case 2:
			{
				BSP_USB2BOTH();
				printf("usb to both\n\r");
				break;
			}
			case 3:
			{
				BSP_USB2PHONE();
				printf("usb to phone\n\r");
				break;
			}
			case 4:
			{
				BSP_BOOST2PHONE();
				printf("boost to phone\n\r");
				break;
			}
			case 5:
			{
				BSP_PHONE2PACK();
				printf("phone to pack\n\r");
				break;
			}
			default:
			{
				BSP_STANDBY();
				printf("default standby\n\r");
				break;
			}
		}
}

void set_pack_status(PACK_INFO *rptr)
{	
	printf("\n\r");
	printf("Attach\tUSB\tPHONE_USB\tUSB_VOL\tPHONE_SOC\tPACK_SOC\n\r");
	printf("%d\t%d\t%d\t%d\t%d\t%d\n\r",rptr->ATTACH,rptr->USB,rptr->PHONE_USB,rptr->PHONE_USB_VOLTAGE,rptr->PHONE_SOC,rptr->PACK_SOC);
	pack_status=get_pack_status(rptr);		
	switch(pack_status){
			case STATUS_STANDBY:
			{
				BSP_STANDBY();
				printf(">>standby\n\r");
				break;
			}
			case STATUS_USB2PACK:
			{
				BSP_USB2PACK();
				printf(">>usb to pack\n\r");
				break;
			}
			case STATUS_USB2BOTH:
			{
				BSP_USB2BOTH();
				printf(">>usb to both\n\r");
				break;
			}
			case STATUS_USB2PHONE:
			{
				BSP_USB2PHONE();
				printf(">>usb to phone\n\r");
				break;
			}
			case STATUS_BOOST2PHONE:
			{
				BSP_BOOST2PHONE();
				printf(">>boost to phone\n\r");
				break;
			}
			case STATUS_PHONE2PACK:
			{
				BSP_PHONE2PACK();
				printf(">>phone to pack\n\r");
				break;
			}
			default:
			{
				BSP_STANDBY();
				printf(">>default\n\r");
				break;
			}
		}
	printf("\n\r");
}

void OS_PreSleepProcessing(uint32_t vParameters)
{
	(void)vParameters;
	
	vParameters = 0;
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	//BSP_I2C1_CLK_DISABLE();
	//BSP_I2C2_CLK_DISABLE(); 
	//BSP_UART1_CLK_DISABLE() ;
	//BSP_UART2_CLK_DISABLE() ;
	//BSP_UART3_CLK_DISABLE() ;
	
	if(pack_status==STATUS_SLEEP)
	{		
		//HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		
	}

}

void OS_PostSleepProcessing(uint32_t vParameters)
{
	(void)vParameters;
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);	
	//BSP_I2C1_CLK_ENABLE();
	//BSP_I2C2_CLK_ENABLE(); 
	//BSP_UART1_CLK_ENABLE() ;
	//BSP_UART2_CLK_ENABLE() ;
	//BSP_UART3_CLK_ENABLE() ;
	
	if(pack_status==STATUS_SLEEP)
	{
		SYSCLKConfig_STOP();
		
	}
	
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable MSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    while(1);
  }
}

void delayms(uint32_t i)
{
	uint32_t j=20000;
	for(;i!=0;i--)
	{
		for(;j!=0;j--)
			;
		j=20000;
	}
}

/**
  * @brief  Retargets the C library printf function to the UART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&Uart2Handle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
