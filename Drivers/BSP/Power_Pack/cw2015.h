/**
  ******************************************************************************
  * @file    cw2015.h
  * @author  
  * @version V1.0.0
  * @date    31-January-2018
  * @brief   
  *          
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CW2015_H
#define __CW2015_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
#include "stm32l4xx_hal.h"
	 
#define CW_ADDRESS 				0xC4
	 
#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)

#define ATHD                    (0x0<<3)        //ATHD = 0%

#define SIZE_BATINFO        64

#define BATTERY_UP_MAX_CHANGE   720             // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE 60             // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min
#define BAT_LOW_INTERRUPT    1
#define CW2015_GET_RRT

/*??????,???????????????????*/
static unsigned char cw_bat_config_info[SIZE_BATINFO] = {
0x17,0xF9,0x6A,0x6D,0x6D,0x6D,0x68,0x65,
0x5C,0x6C,0x57,0x58,0x5E,0x5B,0x48,0x40,
0x35,0x2E,0x27,0x1D,0x24,0x32,0x42,0x4E,
0x21,0x5B,0x0A,0x3D,0x16,0x2D,0x4B,0x58,
0x6E,0x6D,0x6B,0x6D,0x3D,0x1A,0x6B,0x67,
0x15,0x25,0x52,0x87,0x8F,0x91,0x94,0x52,
0x82,0x8C,0x92,0x96,0x63,0x85,0xA8,0xCB,
0x2F,0x7D,0x64,0xA5,0xB5,0x11,0x60,0x09
};

//****************************struct*********************************/
typedef struct tagSTRUCT_CW_BATTERY {
	unsigned char usb_online;
	unsigned int capacity;
	unsigned int voltage;
#ifdef CW2015_GET_RRT
	unsigned int time_to_empty;
#endif
	unsigned char alt;
}STRUCT_CW_BATTERY;

uint8_t BSP_CW_Init(void);
uint8_t BSP_CW_Release_Alrt_Pin(void);
uint8_t BSP_CW_Get_Capacity(void);


#ifdef __cplusplus
}
#endif

#endif /* __CW2015_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

