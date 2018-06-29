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

//#define __UART_FIXED_CMD_LENGTH
	
char RxCommand[RXCOMMANDSIZE];
char PbCommand[RXCOMMANDSIZE];
uint8_t RxBuffer[RXBUFFERSIZE] = {0}; //transmitting byte per byte

char 	temp; //initialisation character
char * s;	

__IO uint8_t RxDataNumber=0; 
__IO uint16_t RxData1=0;
__IO uint16_t RxData2=0;

//#define I2C_TIMING      0x0020098E

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sConfig;
uint16_t   										aADCxConvertedData[4];

UART_HandleTypeDef 						Uart1Handle,Uart2Handle,Uart3Handle;
__IO ITStatus 								UartReady = RESET;

DMA_HandleTypeDef         		DmaHandle;

uint32_t usb_in_voltage, ext_pwr_voltage, acc_id_voltage,vdda;

uint32_t I2c1Timeout = BSP_I2C1_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
uint32_t I2c2Timeout = BSP_I2C2_TIMEOUT_MAX;    /*<! Value of Timeout when I2C1 communication fails */
I2C_HandleTypeDef powerpack_I2c1,powerpack_I2c2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void ADC_Data_Handle(void);
void set_pack_status(PACK_INFO *rptr);

uint8_t p9221_id[2];
uint8_t	p9221_fw_revision[4];
uint8_t smb1381_id[4];

uint8_t key_count=0;
uint8_t reg_add_h,reg_add_l,reg_data,prefix;
uint16_t reg_add16;

uint8_t aRxBuffer[5];

PACK_INFO pack_info;
SMB_TADC smb_tadc;

USB_TYPE *usb_type_temp;

typedef enum
{
	STATUS_STANDBY=0,
	STATUS_USB2PACK,
	STATUS_USB2BOTH,
	STATUS_USB2PHONE,
	STATUS_BOOST2PHONE,
	STATUS_PHONE2PACK
}PACK_STATUS;
	
uint8_t status_pack=STATUS_STANDBY;

typedef enum
{
  THREAD_1 = 0,
  THREAD_2,
	THREAD_3
} Thread_TypeDef;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t UART2_handle, ADC_handle, CHARGING_handle, SMB_handle,LED_handle;
	
/* Private function prototypes -----------------------------------------------*/
static void UART2_task1(void const *argument);
static void ADC_task2(void const *argument);
static void CHG_task3(void const *argument);
static void SMB_task4(void const *argument)	;
static void LED_task5(void const *argument)	;

static xSemaphoreHandle xSemaphore_uart2_int;
static xSemaphoreHandle xCountingSemaphore,xCountingSemaphore_key;
static xQueueHandle xQueue_pack_info;

PACK_STATUS get_pack_status(PACK_INFO *ptr);

static void SYSCLKConfig_STOP(void);
void DecodeReception(void);
void pbfw(void);
void pbsoc(uint8_t soc);
void pbphsoc(uint8_t soc);
void pbusb(uint16_t usb_vol);
void pbwr(uint16_t addr, uint16_t data);
void pbrd(uint16_t addr);
void pbhelp(void);
void pbadc(void);
void pbwrcw(uint16_t addr, uint16_t data);
void pbrdcw(uint16_t addr);
void led_of_soc(uint8_t soc);
void pbpmuxena(uint8_t io);
void pbpmuxenb(uint8_t io);
void pbchgen(uint8_t io);
void pbboosten(uint8_t en);
void pbboost9ven(uint8_t en);
void set_pack_path(uint8_t path);
/* Private functions ---------------------------------------------------------*/

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
  		
  BSP_POWER_PACK_Init();
	printf("Power pack hardware init ok!\n\r");
	
	BSP_SMB_Enable();
	BSP_SMB_Init();
	
	if(BSP_CW_Init()==0)
		printf("CW2015 init ok!\n\r");
	//BSP_USB2SMB();
	pack_info.ATTACH=0;
	pack_info.USB=0;
	pack_info.DCIN=0;
	pack_info.PACK_SOC=BSP_CW_Get_Capacity();
	pack_info.PHONE_SOC=0;	
	
	if(HAL_GPIO_ReadPin(PHONE_ATTACHED_PIN_GPIO_PORT,PHONE_ATTACHED_PIN))
	{
		pack_info.ATTACH=0;
		printf("phone detached!!\n\r");
	}
	else
	{
		pack_info.ATTACH=1;
		printf("phone attached!!\n\r");
	}
	
	if(BSP_SMB_INT_Type()==USB_PLUGIN)
	{
		pack_info.USB=1;
		printf("USB plugin!!\n\r");
		//BSM_SMB_Charging(usb_type_temp);
	}
	else if(BSP_SMB_INT_Type()==USB_PLUGOUT)
	{
		pack_info.USB=0;
		printf("USB plugout!!\n\r");
	}	
	set_pack_status(&pack_info);
	
	/* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Ensure that MSI is wake-up system clock */ 
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
	
	xQueue_pack_info = xQueueCreate( 5, sizeof( PACK_INFO ) );  	
	xCountingSemaphore = xSemaphoreCreateCounting( 10, 0 );
	
  xTaskCreate((TaskFunction_t)UART2_task1,		"task1",3000,NULL,6,	UART2_handle );
	xTaskCreate((TaskFunction_t)ADC_task2,			"task2",300,NULL,2,	ADC_handle );
	xTaskCreate((TaskFunction_t)CHG_task3,			"task3",300,NULL,2,	CHARGING_handle );
	xTaskCreate((TaskFunction_t)SMB_task4,			"task4",300,NULL,7,	SMB_handle );
	xTaskCreate((TaskFunction_t)LED_task5,			"task5",300,NULL,2,	LED_handle );
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
  /* Set transmission flag: transfer complete */
	static portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE;
	if(UartHandle->Instance==BSP_UART2)
	{
		//UartReady = SET; 
		if((xSemaphoreGiveFromISR( xSemaphore_uart2_int, &xHigherPriorityTaskWoken))!=pdPASS)
			printf("UART2 IRQ error\n\r");	
	}
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
    //UartReady = SET; 
	
		/* Toggle LED2 */    
		if(HAL_GPIO_ReadPin(SMB_STAT_PIN_GPIO_PORT,SMB_STAT_PIN)==0)
		{
			if((xSemaphoreGiveFromISR( xCountingSemaphore, &xHigherPriorityTaskWoken_smb))!=pdPASS)
				printf("UART IRQ error\n\r");	
		}
  }

  else if (GPIO_Pin == SMB_SYS_PIN)
  {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == BOOST_OCP_INT_PIN)
  {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
  }

  else if (GPIO_Pin == KEY_PIN)
  {
    if((xSemaphoreGiveFromISR( xCountingSemaphore_key, &xHigherPriorityTaskWoken_key))!=pdPASS)
			printf("KEY IRQ error\n\r");
  }

  else if (GPIO_Pin == PHONE_ATTACHED_PIN)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
		//printf("phone attach pin int!!\n\r");
		if(HAL_GPIO_ReadPin(PHONE_ATTACHED_PIN_GPIO_PORT,PHONE_ATTACHED_PIN))
		{
			pack_info.ATTACH=0;
			printf("phone detached!!\n\r");
		}
		else
		{			
			pack_info.ATTACH=1;
			printf("phone attached!!\n\r");
		}
		xStatus=xQueueSendToBackFromISR( xQueue_pack_info, &pack_info, xHigherPriorityTaskWoken );
		if( xStatus != pdPASS )
		{
			printf("Send to queue error!\n\r");	
		}		
  }

  else if (GPIO_Pin == FG_INT_PIN)
  {
       /* Toggle LED2 */
       BSP_LED_Toggle(LED2);
  }
  
}

/**
  * @brief  UART3 thread 1
  * @param  thread not used
  * @retval None
  */
static void UART2_task1(void const *argument)
{
	(void) argument;
  uint8_t i;
	//portBASE_TYPE xStatus;
	//uint16_t xTicksToWait=100 / portTICK_RATE_MS;
	xSemaphore_uart2_int = xSemaphoreCreateBinary( );
	if(xSemaphore_uart2_int==NULL)
			printf("xSemaphore create fail\n\r");
	for(;;)
	{			
			SET_BIT(Uart2Handle.Instance->CR2, USART_CR2_RTOEN);	//receiver timeout enable
			SET_BIT(Uart2Handle.Instance->CR1, USART_CR1_RTOIE);	//receiver timeout interrupt enable
			Uart2Handle.Instance->RTOR=0x00000016;								//22x bit duration
			
			for(i=0;i<RXBUFFERSIZE;i++)
				*(RxBuffer+i)=0;
			if(HAL_UART_Receive_IT(&Uart2Handle, RxBuffer, RXBUFFERSIZE) != HAL_OK)
				 printf("uart int error\n\r");
			xSemaphoreTake( xSemaphore_uart2_int, portMAX_DELAY );	
		
			DecodeReception();		  
      s = PbCommand;
      
      if (strcmp(s, "pbfw") == 0){pbfw();}
      else if (strcmp(s, "pbsoc") == 0) {pbsoc(RxData1);}
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


static void ADC_task2(void const *argument)
{
	uint32_t vrefint_data,vrefint_cal;
	vrefint_cal=(uint32_t)(*VREFINT_CAL_ADDR);	
	
	for(;;)
	{
	
		vrefint_data=aADCxConvertedData[3];
		vdda=(3000*vrefint_cal)/vrefint_data;	

		usb_in_voltage=(vdda*aADCxConvertedData[0]*11)/4096;
		ext_pwr_voltage=(vdda*aADCxConvertedData[1]*11)/4096;
		acc_id_voltage=(vdda*aADCxConvertedData[2])/4096;
			
		BSP_SMB_TADC(&smb_tadc);
		pack_info.PACK_SOC=BSP_CW_Get_Capacity();
		vTaskDelay(500);
	}
}

static void CHG_task3(void const *argument)
{
	PACK_INFO rptr;	
	portBASE_TYPE xStatus;
	
	for(;;)
	{
		xStatus=xQueueReceive( xQueue_pack_info, &rptr, portMAX_DELAY );
		if (xStatus == pdPASS) 
		{
			set_pack_status(&rptr);
		}
	}
}

static void SMB_task4(void const *argument)
{
	uint8_t apsd_done=0;
	USB_TYPE usb_type;
	SMB_INT int_type;
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;	
	SMB_TADC *smb_tadc;
	uint8_t pulse,i;
	uint32_t max_current,current;
	
	xCountingSemaphore = xSemaphoreCreateCounting( 10, 0 );
	if(xCountingSemaphore==NULL)
			printf("xCountingSemaphore create fail\n\r");
	for(;;)
	{
		xSemaphoreTake( xCountingSemaphore, portMAX_DELAY );
		int_type=BSP_SMB_INT_Type();		
		
		if(int_type==USB_PLUGIN)
		{
			BSP_SMB_USBIN_Exit_Suspend();
			pack_info.USB=1;		
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
			if( xStatus != pdPASS )
			{
				printf("Send to queue error!\n\r");	
			}		
			vTaskDelay(3000);
			
			BSP_SMB_USBIN_Status(&usb_type);
			//BSM_SMB_Charging(&usb_type);
			if(usb_type.CHARGER==QC3_CHARGER)
			{
				i=0;
				max_current=0;
				while(i<20)
				{
					BSP_SMB_BAT_Current_Start();
					vTaskDelay(10);
					current=BSP_SMB_BAT_Current();
					if(current>max_current)
					{
						max_current=current;
						pulse=0x3f&BSP_I2C2_Read(SMB_ADDRESS, 0x130A,I2C_MEMADD_SIZE_16BIT);
					}
					BSM_SMB_QC3_Single_Inc();
					i++;
				}
				if(pulse!=0)
				{
					i=20-pulse;
					printf("QC3 pulse=%d",pulse);
					while(i>0)
					{
						BSM_SMB_QC3_Single_Dec();
						i--;
					}			
				}
			}
			else if(usb_type.CHARGER==QC2_CHARGER)
			{
				BSM_SMB_QC2_Force_5V();
				BSM_SMB_QC2_Force_9V();
				printf("QC2 charger set to 9V");
			}			
			
		}
		else
		{
			pack_info.USB=0;
			xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
			if( xStatus != pdPASS )
			{
				printf("Send to queue error!\n\r");	
			}		
			BSP_SMB_USBIN_Suspend();
		}
			
	}			
}

static void LED_task5(void const *argument)
{
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;	
	xCountingSemaphore_key = xSemaphoreCreateCounting( 10, 0 );
	if(xCountingSemaphore==NULL)
			printf("xCountingSemaphore create fail\n\r");
	for(;;)
	{
		xSemaphoreTake( xCountingSemaphore_key, portMAX_DELAY );	
		xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
		if( xStatus != pdPASS )
		{
			printf("Send to queue error!\n\r");	
		}		
		led_of_soc(pack_info.PACK_SOC);
		
	}			
}

void led_of_soc(uint8_t soc)
{
	uint8_t i=5;
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
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);
	BSP_LED_Off(LED4);
}
/**
  * @brief  Decode recpeption of RBuffer
  * @param  None
  * @retval None
  */
void DecodeReception(void)
{
	uint16_t i;
	char temp;
	
	uint8_t d1_hex=0;
	uint8_t d2_hex=0;
	for(i=0;i<RXCOMMANDSIZE;i++)
	{	
		PbCommand[i]=0;
		RxCommand[i]=0;
	}
	RxDataNumber=0;
	RxData1=0;
	RxData2=0;
	
	for(i=0;RxBuffer[i]!=CR_ASCII_VALUE;i++)
	{
		temp=RxBuffer[i];
		
		if(temp != SPACE_ASCII_VALUE)
		{
			RxCommand[i]=temp;
		}
		else
		{
			RxDataNumber++;
			if(RxDataNumber==1)
			{
				strcpy(PbCommand,RxCommand);
			}
		}
		
		if(RxDataNumber>0)
		{
			if((temp>='0')&&(temp<='9'))
			{
				if(RxDataNumber==1)
				{
					if(d1_hex==1)
						RxData1=(RxData1<<4)+(temp-0x30);
					else
						RxData1=(RxData1*10)+(temp-0x30);
					
				}
				else if(RxDataNumber==2)
				{
					if(d2_hex==1)
						RxData2=(RxData2<<4)+(temp-0x30);
					else
						RxData2=(RxData2*10)+(temp-0x30);
				}	
			}
			else if(temp=='x')
			{
				if(RxDataNumber==1)
				{
					RxData1=0;
					d1_hex=1;					
				}
				else if(RxDataNumber==2)
				{
					RxData2=0;
					d2_hex=1;					
				}	
			}
			else if((temp>='a')&&(temp<='f'))
			{
				if(RxDataNumber==1)
				{
					RxData1=(RxData1<<4)+(temp-0x57);
				}				
				else if(RxDataNumber==2)
				{
					RxData2=(RxData2<<4)+(temp-0x57);
					
				}	
			}
		}
	}
	if(RxDataNumber==0)
			strcpy(PbCommand,RxCommand);
	
	printf(PbCommand);
	if(RxDataNumber==1)
	{
		if(d1_hex==1)
			printf(" 0x%x",RxData1);
		else
			printf(" %d",RxData1);
	}
	else if(RxDataNumber==2)
	{	
		if(d2_hex==1)
			printf(" 0x%x 0x%x",RxData1,RxData2);
		else
			printf(" %d %d",RxData1,RxData2);
	}
	printf("\n\r");

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
	printf("PACK firmware version:%x\n\r",__PACK_BSP_VERSION);
}

void pbadc(void)
{
	printf("CH1 THERM1 NTC:%dC\n\r",smb_tadc.CH1_THERM1_NTC);
	printf("CH2 THERM2 NTC:%dC\n\r",smb_tadc.CH2_THERM2_NTC);
	printf("CH3 DIE TEMP:%dC\n\r",smb_tadc.CH3_DIE_TEMP);
	printf("CH4 BAT CURRENT:%dmA\n\r",smb_tadc.CH4_BAT_CURRENT);
	printf("CH5 BAT VOLTAGE:%dmV\n\r",smb_tadc.CH5_BAT_VOL);
	printf("CH6 VIN CURRENT:%dmA\n\r",smb_tadc.CH6_VIN_CURRENT);
	printf("CH7 VIN VOLTAGE:%dmV\n\r",smb_tadc.CH7_VIN_VOL);
	printf("USB IN VOLTAGE:%dmV\n\r",usb_in_voltage);
	printf("EXT PWR VOLTAGE:%dmV\n\r",ext_pwr_voltage);
	printf("ACC ID VOLTAGE:%dmV\n\r",acc_id_voltage);
	printf("VDDA VOLTAGE:%dmV\n\r",vdda);
	printf("PACK SOC:%d\n\r",pack_info.PACK_SOC);	
}

void pbsoc(uint8_t soc)
{
	portBASE_TYPE xStatus;
	uint16_t xTicksToWait=100 / portTICK_RATE_MS;
	pack_info.PACK_SOC=soc;
	printf("PACK soc:%d\n\r",pack_info.PACK_SOC);
	xStatus=xQueueSendToBack( xQueue_pack_info, &pack_info, xTicksToWait );
	if( xStatus != pdPASS )
	{
		printf("Send to queue error!\n\r");	
	}
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

PACK_STATUS get_pack_status(PACK_INFO *ptr)
{
	
	if((ptr->ATTACH==0)&&	(ptr->USB==0))
		status_pack=STATUS_STANDBY;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==0)&&(ptr->PACK_SOC<=3))		
		status_pack=STATUS_STANDBY;
	else if((ptr->ATTACH==0)&&	(ptr->USB==1))
		status_pack=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==1))		
		status_pack=STATUS_USB2PACK;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC>=90))
		status_pack=STATUS_USB2BOTH;
	else if((ptr->ATTACH==1)&&	(ptr->USB==1)&&(ptr->PHONE_USB==0)&&(ptr->PHONE_SOC<90))
		status_pack=STATUS_USB2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==0)&&(ptr->PACK_SOC>3))
		status_pack=STATUS_BOOST2PHONE;
	else if((ptr->ATTACH==1)&&	(ptr->USB==0)&&(ptr->PHONE_USB==1)&&(ptr->PHONE_SOC>90))
		status_pack=STATUS_PHONE2PACK;
	return status_pack;
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
	PACK_STATUS status;
	printf("Attach\tUSB\tPHONE_USB\tUSB_VOL\tPHONE_SOC\tPACK_SOC\n\r");
	printf("%d\t%d\t%d\t%d\t%d\t%d\n\r",rptr->ATTACH,rptr->USB,rptr->PHONE_USB,rptr->PHONE_USB_VOLTAGE,rptr->PHONE_SOC,rptr->PACK_SOC);
	status=get_pack_status(rptr);		
	switch(status){
			case STATUS_STANDBY:
			{
				BSP_STANDBY();
				printf("standby\n\r");
				break;
			}
			case STATUS_USB2PACK:
			{
				BSP_USB2PACK();
				printf("usb to pack\n\r");
				break;
			}
			case STATUS_USB2BOTH:
			{
				BSP_USB2BOTH();
				printf("usb to both\n\r");
				break;
			}
			case STATUS_USB2PHONE:
			{
				BSP_USB2PHONE();
				printf("usb to phone\n\r");
				break;
			}
			case STATUS_BOOST2PHONE:
			{
				BSP_BOOST2PHONE();
				printf("boost to phone\n\r");
				break;
			}
			case STATUS_PHONE2PACK:
			{
				BSP_PHONE2PACK();
				printf("phone to pack\n\r");
				break;
			}
			default:
			{
				BSP_STANDBY();
				printf("default\n\r");
				break;
			}
		}
}

void OS_PreSleepProcessing(uint32_t vParameters)
{
	(void)vParameters;
	
	vParameters = 0;
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	if(status_pack==STATUS_STANDBY)
	{
		
		HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		//HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	}
	else
		/* Enter Sleep Mode , wake up is done once jumper is put between PA.12 (Arduino D2) and GND */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void OS_PostSleepProcessing(uint32_t vParameters)
{
	(void)vParameters;
	if(status_pack==STATUS_STANDBY)
	{
		SYSCLKConfig_STOP();
		
	}
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
