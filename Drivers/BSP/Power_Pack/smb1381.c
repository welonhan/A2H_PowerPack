/* Includes ------------------------------------------------------------------*/
#include "smb1381.h"
#include "power_pack.h"
#include "smb-reg.h"

#include <stdio.h>

//HAL_StatusTypeDef smb138x_init_hw(void);


void BSP_SMB_Enable(void)
{
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_RESET); 
	
	//osDelay(10);
	HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(SMB_LPMODE_EN_PIN_GPIO_PORT, SMB_LPMODE_EN_PIN, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(SMB_SUSP_PIN_GPIO_PORT, SMB_SUSP_PIN, GPIO_PIN_SET); 
}

void BSP_SMB_Init(void)
{
	HAL_StatusTypeDef status;
	
	//status=smb138x_init_hw();
	//if(status!=HAL_OK)
	//	printf("smb138x init hw error!\n\r");	
	
	//SMB STAT INT use level triggering
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1311, I2C_MEMADD_SIZE_16BIT, 0x00))!=HAL_OK)
		printf("SBM1381 INT level tringgering setting error!\n\r");
	
	//SMB STAT INT polarity high, disable high level int triggering
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1312, I2C_MEMADD_SIZE_16BIT, 0x00))!=HAL_OK)
		printf("SBM1381 INT polarity setting error!\n\r");
	
	//SMB STAT INT polarity low, 1=level low or falling edge
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1313, I2C_MEMADD_SIZE_16BIT, 0xFF))!=HAL_OK)
		printf("SBM1381 INT low level setting error!\n\r");
	
	//disable int
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1316, I2C_MEMADD_SIZE_16BIT, 0xFF))!=HAL_OK)
		printf("SBM1381 INT enable setting error!\n\r");
	
	//enable int
	if((BSP_I2C2_Write(SMB_ADDRESS, 0x1315, I2C_MEMADD_SIZE_16BIT, 0x10))!=HAL_OK)
		printf("SBM1381 INT enable setting error!\n\r");	
	
	//clear pending int
	BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0xff);
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1060, I2C_MEMADD_SIZE_16BIT, 0x24);		//0.9A precharge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1061, I2C_MEMADD_SIZE_16BIT, 0x80);		//3.2A fast charge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1062, I2C_MEMADD_SIZE_16BIT, 0x03);		//300mA end charge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1070, I2C_MEMADD_SIZE_16BIT, 0xBE);		//4.4V float voltage
	BSP_I2C2_Write(SMB_ADDRESS, 0x1074, I2C_MEMADD_SIZE_16BIT, 0x03);		//pre to fast at 3V
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A1, I2C_MEMADD_SIZE_16BIT, 0x00);		//24min precharge safety timer
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A2, I2C_MEMADD_SIZE_16BIT, 0x00);		//192min fast charge safety timer
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A0, I2C_MEMADD_SIZE_16BIT, 0x00);		//enable safety timer
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1090, I2C_MEMADD_SIZE_16BIT, 0x1B);		//JEITA EN CFG
	BSP_I2C2_Write(SMB_ADDRESS, 0x1091, I2C_MEMADD_SIZE_16BIT, 0x1E);		//JEITA FV MINUS 300mV
	BSP_I2C2_Write(SMB_ADDRESS, 0x1092, I2C_MEMADD_SIZE_16BIT, 0x32);		//JEITA FCC MINUS 2000mA
		
	BSP_I2C2_Write(SMB_ADDRESS, 0x1370, I2C_MEMADD_SIZE_16BIT, 0x6C);		//USB IN INPUT CURRENT LIMIT TO 2.7A
	BSP_I2C2_Write(SMB_ADDRESS, 0x1380, I2C_MEMADD_SIZE_16BIT, 0x84);		//EN AICL
	
	//cc
	BSP_I2C2_Write(SMB_ADDRESS, 0x1358, I2C_MEMADD_SIZE_16BIT, 0x00);	
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1360, I2C_MEMADD_SIZE_16BIT, 0x08);	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1362, I2C_MEMADD_SIZE_16BIT, 0x5c);	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1363, I2C_MEMADD_SIZE_16BIT, 0x34);	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1366, I2C_MEMADD_SIZE_16BIT, 0x03);	
	
	//suspend
	BSP_I2C2_Write(SMB_ADDRESS, 0x1340, I2C_MEMADD_SIZE_16BIT, 0x1);	
	
	//enabel TADC
	BSP_I2C2_Write(SMB_ADDRESS, 0x3646, I2C_MEMADD_SIZE_16BIT, 0x80);	
	
	//enable charging
	BSP_I2C2_Write(SMB_ADDRESS, 0x1042, I2C_MEMADD_SIZE_16BIT, 0x01);
}

void BSP_SMB_En_APSD_HVDCP(void)
{
	//charging enable source by command register
	BSP_I2C2_Write(SMB_ADDRESS, 0x1051, I2C_MEMADD_SIZE_16BIT, 0x02);
	
	//enable APSD, HVDCP
	BSP_I2C2_Write(SMB_ADDRESS, 0x1362, I2C_MEMADD_SIZE_16BIT, 0x5c);	
}

uint8_t BSP_SMB_APSD_Done(void)
{
	uint8_t temp;
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1307,I2C_MEMADD_SIZE_16BIT);
	if(temp&0x01)
	{
		printf("APSD done\n\r");
		return 1;
	}	
	else
		return 0;
}

uint8_t BSP_SMB_SDP_Done(void)
{
	uint8_t temp;
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1307,I2C_MEMADD_SIZE_16BIT);
	if(temp&0x10)
	{
		printf("SDP done\n\r");
		return 1;
	}	
	else
		return 0;
}

void BSP_SMB_USBIN_Limit(uint16_t limit_mA)
{
	uint8_t temp;
	temp=(uint8_t)limit_mA/25;
	BSP_I2C2_Write(SMB_ADDRESS, 0x1370, I2C_MEMADD_SIZE_16BIT, temp);
	
}

void BSP_SMB_USBIN_Suspend(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1340, I2C_MEMADD_SIZE_16BIT, 0x1);	
}

void BSP_SMB_USBIN_Exit_Suspend(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1340, I2C_MEMADD_SIZE_16BIT, 0x0);	
}

void BSP_SMB_HC_Config(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1060, I2C_MEMADD_SIZE_16BIT, 0x24);		//0.9A precharge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1061, I2C_MEMADD_SIZE_16BIT, 0x80);		//3.2A fast charge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1062, I2C_MEMADD_SIZE_16BIT, 0x03);		//300mA end charge
	BSP_I2C2_Write(SMB_ADDRESS, 0x1070, I2C_MEMADD_SIZE_16BIT, 0xBE);		//4.4V float voltage
	BSP_I2C2_Write(SMB_ADDRESS, 0x1074, I2C_MEMADD_SIZE_16BIT, 0x03);		//pre to fast at 3V
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A1, I2C_MEMADD_SIZE_16BIT, 0x00);		//24min precharge safety timer
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A2, I2C_MEMADD_SIZE_16BIT, 0x00);		//192min fast charge safety timer
	BSP_I2C2_Write(SMB_ADDRESS, 0x10A0, I2C_MEMADD_SIZE_16BIT, 0x00);		//enable safety timer
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1090, I2C_MEMADD_SIZE_16BIT, 0x1B);		//JEITA EN CFG
	BSP_I2C2_Write(SMB_ADDRESS, 0x1091, I2C_MEMADD_SIZE_16BIT, 0x1E);		//JEITA FV MINUS 300mV
	BSP_I2C2_Write(SMB_ADDRESS, 0x1092, I2C_MEMADD_SIZE_16BIT, 0x32);		//JEITA FCC MINUS 2000mA
		
	BSP_I2C2_Write(SMB_ADDRESS, 0x1370, I2C_MEMADD_SIZE_16BIT, 0x6C);		//USB IN INPUT CURRENT LIMIT TO 2.7A
	BSP_I2C2_Write(SMB_ADDRESS, 0x1380, I2C_MEMADD_SIZE_16BIT, 0x14);		//EN AICL
	
	//Type -c
	
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1366, I2C_MEMADD_SIZE_16BIT, 0x03);
	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1042, I2C_MEMADD_SIZE_16BIT, 0x01);		//EN CHARGE	
}

void BSP_SMB_USBIN_HC_Mode(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1366, I2C_MEMADD_SIZE_16BIT, 0x0);	
}

uint8_t BSP_SMB_USBIN_Status(USB_TYPE *usb_type)
{
	uint8_t temp;
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1306,I2C_MEMADD_SIZE_16BIT);
	if(temp&0x01)
	{
		printf("USBIN_5V\n\r");
		usb_type->USBIN=USBIN_5V;
	}	
	else if(temp&0x02)
	{
		printf("USBIN_5V_TO_9V\n\r");
		usb_type->USBIN=USBIN_5V_TO_9V;
	}	
	else if(temp&0x04)
	{
		printf("USBIN_5V_TO_12V\n\r");
		usb_type->USBIN=USBIN_5V_TO_12V;
	}	
	else if(temp&0x08)
	{
		printf("USBIN_9V\n\r");
		usb_type->USBIN=USBIN_9V;
	}	
	else if(temp&0x10)
	{
		printf("USBIN_9V_TO_12V\n\r");
		usb_type->USBIN=USBIN_9V_TO_12V;
	}	
	else if(temp&0x20)
	{
		printf("USBIN_12V\n\r");
		usb_type->USBIN=USBIN_12V;
	}	
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1308,I2C_MEMADD_SIZE_16BIT);
	if(temp&0x01)
	{
		printf("SDP_CHARGER\n\r");
		usb_type->CHARGER=SDP_CHARGER;
	}	
	else if(temp&0x02)
	{
		printf("OCP_CHARGER\n\r");
		usb_type->CHARGER=OCP_CHARGER;
	}	
	else if(temp&0x04)
	{
		printf("CDP_CHARGER\n\r");
		usb_type->CHARGER=CDP_CHARGER;
	}	
	else if(temp&0x08)
	{
		printf("DCP_CHARGER\n\r");
		usb_type->CHARGER=DCP_CHARGER;
		if(temp&0x20)
		{
			printf("QC2_CHARGER\n\r");
			usb_type->CHARGER=QC2_CHARGER;
		}	
		else if(temp&0x40)
		{
			printf("QC3_CHARGER\n\r");
			usb_type->CHARGER=QC3_CHARGER;
		}	
	}	
	else if(temp&0x10)
	{
		printf("FLOAT_CHARGER\n\r");
		usb_type->CHARGER=FLOAT_CHARGER;
	}	
	
	else if(temp&0x80)
	{
		printf("ICL override\n\r");
		usb_type->CHARGER=ICL_OVERRIDE;
	}	
}

SMB_INT BSP_SMB_INT_Type(void)
{
		uint8_t temp;	
		
		BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0xff);
		temp=BSP_I2C2_Read(SMB_ADDRESS, 0x1310,I2C_MEMADD_SIZE_16BIT);
		printf("USBIN STATUS=0x%x\n\r",temp);				
		
		if(temp&0x01)
		{	
			printf("USB collapse!\n\r");	
				return USB_COLLAPSE;	
			//if((BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0x01))!=HAL_OK)
			//	printf("clear USB collapse latched error!\n\r");	
		}	
		/*	
		if(temp&0x02)
		{	
			printf("USBIN cross 3V6!\n\r");	
			//if((BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0x02))!=HAL_OK)
				//printf("clearUSBIN cross 3V6 latched error!\n\r");	
		}
		if(temp&0x04)
		{	
			printf("USBIN cross UV!\n\r");	
			//if((BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0x04))!=HAL_OK)
				//printf("clear USBIN cross UV latched error!\n\r");	
		}
		if(temp&0x08)
		{	
			printf("USBIN over voltage!\n\r");	
			//if((BSP_I2C2_Write(SMB_ADDRESS, 0x1314, I2C_MEMADD_SIZE_16BIT, 0x08))!=HAL_OK)
				//printf("clear USBIN over voltage latched error!\n\r");	
		}
			*/
		if(temp&0x10)
		{	
			printf("USB plug in!\n\r");	
				return USB_PLUGIN;			
		}
		else
		{	
			printf("USB plug out!\n\r");	
			return USB_PLUGOUT;			
		}
		
		if(temp&0x20)
		{	
			printf("USBIN source change !\n\r");
			return USB_SRC_CHANGE;			
		}	
		else if(temp&0x40)
		{	
			printf("USBIN change in input current limit !\n\r");	
			return USB_CURRENT_LIMIT;			
		}
		else if(temp&0x80)
		{	
			printf("USB cc pin change!\n\r");	
			return USB_CC_CHANGE;			
		}
}

void BSM_SMB_Charging(USB_TYPE *usb_type)
{
	uint8_t apsd_done,temp;
	//USB_TYPE usb_type;
		
						
	switch(usb_type->CHARGER)
	{
		case SDP_CHARGER:
		{
			if(BSP_SMB_SDP_Done())
				BSP_SMB_USBIN_Limit(500);
			else
				BSP_SMB_USBIN_Suspend();
			break;
		}							
		case OCP_CHARGER:
		{
			BSP_SMB_USBIN_Limit(500);							
			//BSP_SMB_USBIN_HC_Mode();
			break;
		}
		case CDP_CHARGER:
		{
			BSP_SMB_USBIN_Limit(1500);							
			//BSP_SMB_USBIN_HC_Mode();
			break;
		}		
		case DCP_CHARGER:
		{
			BSP_SMB_USBIN_Limit(2000);							
			//BSP_SMB_USBIN_HC_Mode();
			break;
		}		
		case FLOAT_CHARGER:
		{
			BSP_SMB_USBIN_Limit(500);							
			//BSP_SMB_USBIN_HC_Mode();
			break;
		}
		case QC2_CHARGER:
		{
			BSP_SMB_USBIN_Limit(3000);							
			//BSP_SMB_USBIN_HC_Mode();			
			break;
		}
		case QC3_CHARGER:
		{
			BSP_SMB_USBIN_Limit(3000);							
			//BSP_SMB_USBIN_HC_Mode();
			
			break;
		}
		case ICL_OVERRIDE:
		{
			BSP_SMB_USBIN_Suspend();
			break;
		}
		default:
		{
			BSP_SMB_USBIN_Limit(500);							
			//BSP_SMB_USBIN_HC_Mode();
			break;						
		}
	}
	//BSP_SMB_HC_Config();		

	
	BSP_I2C2_Write(SMB_ADDRESS, 0x1340, I2C_MEMADD_SIZE_16BIT, 0x0);
}

void BSM_SMB_QC3_Single_Inc(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1343, I2C_MEMADD_SIZE_16BIT, 0x01);	
}

void BSM_SMB_QC3_Single_Dec(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1343, I2C_MEMADD_SIZE_16BIT, 0x02);	
}

void BSM_SMB_QC2_Force_5V(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1343, I2C_MEMADD_SIZE_16BIT, 0x04);	
}

void BSM_SMB_QC2_Force_9V(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1343, I2C_MEMADD_SIZE_16BIT, 0x10);	
}

void BSM_SMB_QC2_Force_12V(void)
{
	BSP_I2C2_Write(SMB_ADDRESS, 0x1343, I2C_MEMADD_SIZE_16BIT, 0x20);	
}




HAL_StatusTypeDef smblib_masked_write(uint16_t reg, uint8_t bits, uint8_t data)
{
	uint8_t temp;
	HAL_StatusTypeDef status = HAL_OK;
	
	temp=BSP_I2C2_Read(SMB_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT);
	if(data == 0)
	{
		temp=temp&(~(1<<bits));
	}
	else if (data==1)
	{
		temp=temp|(1<<bits);
	}
	
	status=BSP_I2C2_Write(SMB_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, temp);	
	
	return status;	
}

void BSP_SMB_Disable(void)
{
  HAL_GPIO_WritePin(SMB_RESET_PIN_GPIO_PORT, SMB_RESET_PIN, GPIO_PIN_RESET); 
}

uint8_t BSP_SMB_Get_ID(uint8_t *smb_revid)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	status=BSP_I2C2_ReadBuffer(SMB_ADDRESS, 0x102, I2C_MEMADD_SIZE_16BIT, smb_revid, 4);
	if(status == HAL_OK)
	{
			return 0;
	}
	else 
		return 1;
}


void BSP_SMB_TADC(SMB_TADC *smb_tadc)
{
	uint8_t temp;
	uint32_t adc;
	uint32_t num;
	//enabel TADC
	BSP_I2C2_Write(SMB_ADDRESS, 0x3646, I2C_MEMADD_SIZE_16BIT, 0x80);	
	
	//request TADC
	BSP_I2C2_Write(SMB_ADDRESS, 0x3651, I2C_MEMADD_SIZE_16BIT, 0x7f);	
	
	do
	{
		temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3606, I2C_MEMADD_SIZE_16BIT);
	}
	while(temp!=0x7f);
	
	//CH1 THERMESTOR1 NTC
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3661, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3660, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH1_THERM1_NTC=adc*171/1000;
	
	//CH2 THERMESTOR2 NTC
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3663, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3662, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH2_THERM2_NTC=adc*6278/10000;
	
	//CH3 die temperature
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3665, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3664, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH3_DIE_TEMP=adc*122/100;
	
	//CH4 BAT current
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3667, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3666, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH4_BAT_CURRENT=adc*196/10;
	
	//CH5 BAT voltage
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3669, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3668, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH5_BAT_VOL=adc*489/100;
	
	//CH6 VIN current
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3671, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3670, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH6_VIN_CURRENT=adc*140/10;
	
	//CH7 VIN voltage
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3673, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3672, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	smb_tadc->CH7_VIN_VOL=adc*244/10;
}

void BSP_SMB_BAT_Current_Start(void)
{
	//request TADC
	BSP_I2C2_Write(SMB_ADDRESS, 0x3651, I2C_MEMADD_SIZE_16BIT, 0x10);		
}

uint32_t BSP_SMB_BAT_Current(void)
{
	uint8_t temp;
	uint32_t adc;
	uint32_t current;	
	
	//CH4 BAT current
	adc=0;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3667, I2C_MEMADD_SIZE_16BIT);
	adc=temp;
	temp=BSP_I2C2_Read(SMB_ADDRESS, 0x3666, I2C_MEMADD_SIZE_16BIT);
	adc=(adc<<8)+temp;
	current=adc*196/10;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
