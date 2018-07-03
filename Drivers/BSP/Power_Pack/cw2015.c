/* Includes ------------------------------------------------------------------*/
#include "smb1381.h"
#include "power_pack.h"
#include "smb-reg.h"

#include <stdio.h>

#include "cw2015.h"

uint8_t CHARGE = 0; //?????????,1?????,0??????
uint8_t no_charger_full_jump =0;
unsigned int allow_no_charger_full =0;
unsigned int allow_charger_always_zero =0;
uint8_t if_quickstart =0;
uint8_t reset_loop =0;

extern I2C_HandleTypeDef powerpack_I2c2;
extern uint32_t I2c2Timeout;

/*????????,??????????include Cellwise CW201x Driver for MCU.h??,??extern??cw_bat*/
STRUCT_CW_BATTERY   cw_bat;

////////////////////////////////////////////////////////////////////////////////////
////global function: 'cw_bat_work()'  and  'cw_bat_init()'                      ////
////'cw_bat_work()'need be called by main.c in every second                     ////
////'cw_bat_init()'need be called by main.c in system-init after power on reset ////
////////////////////////////////////////////////////////////////////////////////////

//void delay_us(uint8_t us);	
uint8_t cw_read(uint8_t PointReg,uint8_t *pData);
uint8_t cw_write(uint8_t PointReg,uint8_t *pData);
uint8_t cw_read_word(uint8_t point_reg,uint8_t *r_pdata, unsigned int length);

void CW_Delay10us(uint32_t i)
{
	uint32_t j,k;
	for(k=0;k<i;k++)		//delay 100uS
	{
		for(j=0;j<800;j++)		//delay 100uS
		;	
	}
}

uint8_t cw_read(uint8_t PointReg,uint8_t *pData)
{
	uint8_t ret;
	if(HAL_I2C_Mem_Read(&powerpack_I2c2, CW_ADDRESS, PointReg, I2C_MEMADD_SIZE_8BIT, pData, 1, I2c2Timeout)!=HAL_OK)
		ret=1;
  else
		ret=0;
	return ret;	
}

uint8_t cw_read_word(uint8_t point_reg,uint8_t *r_pdata, unsigned int length)
{
	uint8_t ret,temp;
	if(HAL_I2C_Mem_Read(&powerpack_I2c2, CW_ADDRESS, point_reg, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2c2Timeout)!=HAL_OK)
		ret=1;
  else
	{
		*r_pdata=temp;
		ret=0;
	}
	if(HAL_I2C_Mem_Read(&powerpack_I2c2, CW_ADDRESS, point_reg, I2C_MEMADD_SIZE_8BIT, &temp, 1, I2c2Timeout)!=HAL_OK)
		ret=1;
  else
	{
		*(r_pdata+1)=temp;
		ret=0;
	}
	return ret;
}
uint8_t cw_write(uint8_t PointReg,uint8_t *pData)
{
	uint8_t ret;
	if((BSP_I2C2_Write(CW_ADDRESS, PointReg, I2C_MEMADD_SIZE_8BIT,  *pData))!=HAL_OK)
		ret=1;
	else
		ret=0;
	return ret;
}

uint8_t cw_update_config_info(void)
{
	uint8_t ret = 0;
	uint8_t i;
	uint8_t reset_val;
	uint8_t reg_val;
	
	/* make sure no in sleep mode */
	ret = cw_read(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP)
	{
		return 2;
	}
	/* update new battery info */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		reg_val = cw_bat_config_info[i];
		ret = cw_write(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
	}

	/* readback & check */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		ret = cw_read(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
		if(reg_val != cw_bat_config_info[i])
		{
			return 3;
		}
	}
	/* set cw2015/cw2013 to use new battery info */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	reg_val &= 0x07;                /* clear ATHD */
	reg_val |= ATHD;                /* set ATHD */
	ret = cw_write(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	/* reset */
	reset_val = MODE_NORMAL;
	reg_val = MODE_RESTART;
	ret = cw_write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	CW_Delay10us(10);		//delay 100uS
				
	ret = cw_write(REG_MODE, &reset_val);
	if(ret)
	{
		return 1;
	}   
	return 0;
}

/*
need run every time power up
1: i2c error
2: sleep mode
3: profile changed
4: soc error
*/
uint8_t BSP_CW_Init(void)
{
	uint8_t ret;
	uint8_t i;
	uint8_t reg_val = MODE_NORMAL;
	
	/* wake up cw2015/13 from sleep mode */
	ret = cw_write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}

	/* check ATHD if not right */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & 0xf8) != ATHD)
	{
		//"the new ATHD need set"
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(REG_CONFIG, &reg_val);
		if(ret)
		{
			return 1;
		}
	}
	
	/* check config_update_flag if not right */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if(!(reg_val & CONFIG_UPDATE_FLG))
	{
		//"update flag for new battery info need set"
		ret = cw_update_config_info();
		if(ret)
		{
			return ret;
		}
	}
	else
	{
		for(i = 0; i < SIZE_BATINFO; i++)
		{ 
			ret = cw_read(REG_BATINFO +i, &reg_val);
			if(ret)
			{
				return 1;
			}
			if(cw_bat_config_info[i] != reg_val)
			{
				break;
			}
		}
		if(i != SIZE_BATINFO)
		{
			//"update flag for new battery info need set"
			ret = cw_update_config_info();
			if(ret)
			{
				return ret;
			}
		}
	}
	/* check SOC if not eqaul 255 */
	for (i = 0; i < 30; i++) {
		CW_Delay10us(10000);		//delay 100mS
			
		ret = cw_read(REG_SOC, &reg_val);
		if (ret)
			return 1;
		else if (reg_val <= 100) 
			break;		
    }
	
    if (i >=30){
        reg_val = MODE_SLEEP;
        ret = cw_write(REG_MODE, &reg_val);
        // "cw2015/cw2013 input unvalid power error_2\n";
        return 4;
    } 
	return 0;
}

#ifdef BAT_LOW_INTERRUPT
/*release alrt pin*/
uint8_t BSP_CW_Release_Alrt_Pin(void)
{
	uint8_t ret = 0;
	uint8_t reg_val;
	uint8_t alrt;
	ret = cw_read(REG_RRT_ALERT, &reg_val);
	if (ret) {
			return 1;
	}
	alrt = reg_val & 0x80;
	
	reg_val = reg_val & 0x7f;
	ret = cw_write(REG_RRT_ALERT, &reg_val);
	if(ret) {
			return 1;
	}
	
	return 0;
}


uint8_t cw_update_athd()
{
	uint8_t ret = 0;
	uint8_t reg_val;
	char new_athd = 0;
	
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	new_athd = (reg_val >> 3) - 1;
	if(new_athd <= 0){
		new_athd = 0;
	}
	new_athd = new_athd << 3;

	//"the new ATHD need set"
	reg_val &= 0x07;    /* clear ATHD */
	reg_val |= new_athd;    /* set new ATHD */
	ret = cw_write(REG_CONFIG, &reg_val);
	if(ret)
	{
			return 1;
	}
	return 0;
}

#endif

uint8_t cw_por(void)
{
	uint8_t ret = 0;
	uint8_t reset_val = 0;
	reset_val = MODE_SLEEP;             
	ret = cw_write(REG_MODE, &reset_val);
	if (ret)
		return 1;
	CW_Delay10us(10); //delay 100us
	
	reset_val = MODE_NORMAL;
	ret = cw_write(REG_MODE, &reset_val);
	if (ret)
		return 1;
	CW_Delay10us(10); //delay 100us
	
	ret = BSP_CW_Init();
	if (ret) 
		return ret;
	return 0;
}

uint8_t BSP_CW_Get_Capacity(void)
{
	uint8_t ret = 0;
	uint8_t allow_capacity;
	uint8_t reg_val;
	//uint8_t reset_val;
	uint8_t cw_capacity;
	//int charge_time;

	ret = cw_read(REG_SOC, &reg_val);
	if(ret)
	{
		return 1;
	}
        
	cw_capacity = reg_val;
	
	if ((cw_capacity < 0) || (cw_capacity > 100)) {
                // "get cw_capacity error; cw_capacity = %d\n"
        reset_loop++;
		if (reset_loop >5) { 
			ret = cw_por(); //por ic
			if(ret)
				return -1;
			reset_loop =0;               
		}                   
        return cw_bat.capacity;
    }else {
        reset_loop =0;
    }
	
	
	if(((cw_bat.usb_online == 1) && (cw_capacity == (cw_bat.capacity - 1)))
			|| ((cw_bat.usb_online == 0) && (cw_capacity == (cw_bat.capacity + 1))))
	{
		// modify battery level swing
		if(!((cw_capacity == 0 && cw_bat.capacity <= 2)||(cw_capacity == 100 && cw_bat.capacity == 99)))
		{			
			cw_capacity = cw_bat.capacity;
		}
	}
			
	if((cw_bat.usb_online == 1) && (cw_capacity >= 95) && (cw_capacity <= cw_bat.capacity) )
	{     
		// avoid not charge full
		allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_UP_MAX_CHANGE)
		{
			allow_capacity = cw_bat.capacity + 1; 
			cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			no_charger_full_jump =1;
			allow_no_charger_full =0;
		}
		else if(cw_capacity <= cw_bat.capacity)
		{
			cw_capacity = cw_bat.capacity; 
		}
	}

    else if((cw_bat.usb_online == 0) && (cw_capacity <= cw_bat.capacity ) && (cw_capacity >= 90) && (no_charger_full_jump == 1))
	{
		// avoid battery level jump to CW_BAT
		if(cw_bat.usb_online == 0) 
		   allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_DOWN_MIN_CHANGE)
		{
			allow_capacity = cw_bat.capacity - 1;
			allow_no_charger_full =0; 
			if (cw_capacity >= allow_capacity)
			{
				no_charger_full_jump =0;
			}
			else
			{
				cw_capacity = (allow_capacity > 0) ? allow_capacity : 0;
			}
		}
		else if(cw_capacity <= cw_bat.capacity)
		{
			cw_capacity = cw_bat.capacity;
		}
	}
	else
    {
  		allow_no_charger_full =0;
    }
	
	
	if((cw_bat.usb_online > 0) && (cw_capacity == 0))
	{		  
		allow_charger_always_zero++;
		if((allow_charger_always_zero >= BATTERY_DOWN_MIN_CHANGE_SLEEP) && (if_quickstart == 0))
		{
            ret = cw_por(); //por ic
			if(ret){
				return -1;
			}
			if_quickstart = 1;
			allow_charger_always_zero =0;
		}
	}
	else if((if_quickstart == 1)&&(cw_bat.usb_online == 0))
	{
		if_quickstart = 0;
	}

	return(cw_capacity);
}

uint16_t cw_get_vol(void)
{
	uint8_t ret = 0;
	uint8_t get_ad_times = 0;
	uint8_t reg_val[2] = {0 , 0};
	unsigned long ad_value = 0;
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;

	for(get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ret = cw_read_word(REG_VCELL, &reg_val[0],2);
		if(ret)
		{
			return 1;
		}
		ad_buff = (reg_val[0] << 8) + reg_val[1];

		if(get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if(ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if(ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = ad_value  * 305 / 1000;
	return(ad_value);       //14?ADC???
}

#ifdef CW2015_GET_RRT
uint16_t cw_get_time_to_empty(void)
{
        signed char ret;
        uint8_t reg_val;
        unsigned int value16;

        ret = cw_read(REG_RRT_ALERT, &reg_val);
        if (ret)
                return 0;

        value16 = (unsigned int)reg_val;

        ret = cw_read(REG_RRT_ALERT + 1, &reg_val);
        if (ret)
                return 0;

        value16 = ((value16 << 8) + reg_val) & 0x1fff;
       
        return value16;
}
#endif

void update_capacity(void)
{
	int cw_capacity;
	cw_capacity = BSP_CW_Get_Capacity();
	if((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat.capacity != cw_capacity))
	{       
		cw_bat.capacity = cw_capacity;
	}
}


void update_vol(void)
{
	unsigned int cw_voltage;
	cw_voltage = cw_get_vol();
	if(cw_voltage == 1){
		//read voltage error
		cw_bat.voltage = cw_bat.voltage;
	}else if(cw_bat.voltage != cw_voltage)
	{
		cw_bat.voltage = cw_voltage;
	}
}

#ifdef CW2015_GET_RRT
static void update_time_to_empty(void)
{
	unsigned int rrt;
	rrt = (unsigned int)cw_get_time_to_empty();
	if((rrt > 0) && (cw_bat.time_to_empty != rrt))
	{
		cw_bat.time_to_empty = rrt;
	}
}
#endif
/*
static void update_alt(void)
{
	signed int alt;
	alt = cw_get_alt();
	if ((rrt >= 0) && (cw_bat.alt != alt))
	{
		cw_bat.alt = (unsigned int)alt;
	}       
}
*/

void update_usb_online(void)
{
	if(CHARGE == 1) 
	{
		cw_bat.usb_online = 1;
	}
	else
	{
		cw_bat.usb_online = 0;
	}
}

////////////////////////////////////////MCU??????//////////////////////////////////////////
void cw_bat_work(void)
{
	update_usb_online();
	update_capacity();
	update_vol();
#ifdef CW2015_GET_RRT
	update_time_to_empty();
#endif
}

/*
static void cw_bat_gpio_init(void)
{
     
     usb_det_pin -- init
     alt_pin  -- init
 
     return 0;
}
*/

///////////////////////////////////////MCU????????.//////////////////////////////////////
uint8_t cw_bat_init(void)
{
	uint8_t ret;
	uint8_t loop = 0;
	//cw_bat_gpio_init();
	
	ret = BSP_CW_Init();
	while((loop++ < 200) && (ret != 0))
	{
		ret = BSP_CW_Init();
	}
	
	cw_bat.usb_online = 0;
	cw_bat.capacity = 2;
	cw_bat.voltage = 0;
#ifdef CW2015_GET_RRT
	cw_bat.time_to_empty = 0;
#endif
	cw_bat.alt = 0;
	
	return ret;	
}




