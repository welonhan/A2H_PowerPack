/**
  ******************************************************************************
  * @file    p9221.h
  * @author  
  * @version V1.0.0
  * @date    31-January-2018
  * @brief   
  *          
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMB1381_H
#define __SMB1381_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal.h"
	 
#define SMB_ADDRESS 				0x10
	 
typedef enum
{
	USBIN_5V=0,
	USBIN_5V_TO_9V,
	USBIN_5V_TO_12V,
	USBIN_9V,
	USBIN_9V_TO_12V,
	USBIN_12V
}USBIN_STATUS;

typedef enum
{
	SDP_CHARGER=0,
	OCP_CHARGER,
	CDP_CHARGER,
	DCP_CHARGER,
	FLOAT_CHARGER,
	QC2_CHARGER,
	QC3_CHARGER,
	ICL_OVERRIDE
}APSD_STATUS;

typedef enum
{
	USB_PLUGIN=0,
	USB_PLUGOUT,
	USB_SRC_CHANGE,
	USB_CURRENT_LIMIT,
	USB_CC_CHANGE,
	USB_COLLAPSE,
	DCIN_PLUGIN,
	DCIN_PLUGOUT
}SMB_INT;

typedef struct
{
	USBIN_STATUS	USBIN;	
	APSD_STATUS CHARGER;
}USB_TYPE;

typedef struct
{
	uint32_t CH1_THERM1_NTC;
	uint32_t CH2_THERM2_NTC;
	int32_t CH3_DIE_TEMP;
	int32_t CH4_BAT_CURRENT;
	int32_t CH5_BAT_VOL;
	int32_t CH6_VIN_CURRENT;	
	int32_t CH7_VIN_VOL;
}SMB_TADC;

typedef struct
{
	uint8_t USBIN;
	uint8_t DCIN;
}SMB_IN_STATE;

void BSP_SMB_Enable(void);
void BSP_SMB_Disable(void);	 
void BSP_SMB_Init(void);
uint8_t BSP_SMB_Get_ID(uint8_t *smb_revid);
void BSP_SMB_En_APSD_HVDCP(void);
uint8_t BSP_SMB_APSD_Done(void);
void BSP_SMB_USBIN_Status(USB_TYPE *usb_type);
uint8_t BSP_SMB_SDP_Done(void);
void BSP_SMB_USBIN_Limit(uint16_t limit_mA);
//void BSP_SMB_USBIN_Susp(void);
void BSP_SMB_USBIN_HC_Mode(void);
void BSP_SMB_HC_Config(void);
//SMB_INT BSP_SMB_INT_Type(void);
void BSP_SMB_TADC(SMB_TADC *smb_tadc);
void BSP_SMB_Charging(USB_TYPE *usb_type);
void BSP_SMB_QC3_Single_Inc(void);
void BSP_SMB_QC3_Single_Dec(void);
void BSP_SMB_QC2_Force_5V(void);
void BSP_SMB_QC2_Force_9V(void);
void BSP_SMB_QC2_Force_12V(void);
void BSP_SMB_USBIN_Exit_Suspend(void);
void BSP_SMB_USBIN_Suspend(void);
int32_t BSP_SMB_BAT_Current(void);
void BSP_SMB_BAT_Current_Start(void);
void BSP_SMB_INT_Type(SMB_IN_STATE *in_state);
void BSP_SMB_DCIN_Exit_Suspend(void);
void BSP_SMB_DCIN_Suspend(void);
uint8_t BSP_SMB_High_Duty(void);
uint8_t BSP_SMB_USB_Detect(void);
void BSP_SMB_TADC_Start(void);
void BSP_SMB_DCIN_INT_Enable(void);
void BSP_SMB_DCIN_INT_Disable(void);	

#ifdef __cplusplus
}
#endif

#endif /* __SMB1381_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

