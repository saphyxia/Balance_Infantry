/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

typedef struct 
{
    uint32_t id;
    
    /*低电压警告*/
    uint8_t low_power;

    /*充放电状态*/
    uint8_t charge_state;

    /*底盘功率*/
    int16_t chassis_power;

    /*电量*/
    int16_t ele_quantity;

    /*电容电压*/
    int16_t cap_v;
}SuperCap_ReceivePacket_Typedef;

extern SuperCap_ReceivePacket_Typedef SuperCap_ReceivePacket;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TypeDef *Instance,uint32_t StdId,uint8_t data[8],uint8_t length);

#endif //BSP_CAN_H

