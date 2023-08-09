/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "referee_info.h"

uint8_t MotorCAN_FramInfo[4][8];
uint8_t SuperCap_TransmitPacket[8];

static void Referee_Info_Report(void);

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  TickType_t systick = 0;
	
  /* Infinite loop */
  for(;;)
  {
	systick = osKernelSysTick();

	MotorCAN_FramInfo[0][0] = 0xA1;
	MotorCAN_FramInfo[0][4] = (uint8_t)(Control_Info.SendValue[0][Left_Wheel]);
	MotorCAN_FramInfo[0][5] = (uint8_t)(Control_Info.SendValue[0][Left_Wheel] >> 8);
	USER_CAN_TxMessage(CAN1,0x144,MotorCAN_FramInfo[0],8);

	MotorCAN_FramInfo[1][0] = 0xA1;
	MotorCAN_FramInfo[1][4] = (uint8_t)(Control_Info.SendValue[0][Right_Wheel]);
	MotorCAN_FramInfo[1][5] = (uint8_t)(Control_Info.SendValue[0][Right_Wheel] >> 8);
	USER_CAN_TxMessage(CAN1,0x143,MotorCAN_FramInfo[1],8);

	MotorCAN_FramInfo[2][DJI_Motor[Left_Momentum].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[1][Left_Momentum] >> 8);
	MotorCAN_FramInfo[2][DJI_Motor[Left_Momentum].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[1][Left_Momentum]);
	MotorCAN_FramInfo[2][DJI_Motor[Right_Momentum].CANFrame.FrameIndex]   = (uint8_t)(Control_Info.SendValue[1][Right_Momentum] >> 8);
	MotorCAN_FramInfo[2][DJI_Motor[Right_Momentum].CANFrame.FrameIndex+1] = (uint8_t)(Control_Info.SendValue[1][Right_Momentum]);
	USER_CAN_TxMessage(CAN1,DJI_Motor[Right_Momentum].CANFrame.TxStdId,MotorCAN_FramInfo[2],DJI_Motor[Right_Momentum].CANFrame.FrameIndex+2);

	SuperCap_TransmitPacket[0] = (uint8_t)(Control_Info.IF_SuperCap_Eeable);
	SuperCap_TransmitPacket[1] = 0;
	SuperCap_TransmitPacket[2] = (uint8_t)(Referee_Info.power_heat.chassis_power_buffer>>8);
	SuperCap_TransmitPacket[3] = (uint8_t)(Referee_Info.power_heat.chassis_power_buffer);
	SuperCap_TransmitPacket[4] = 0;
	SuperCap_TransmitPacket[5] = 0;
	SuperCap_TransmitPacket[6] = (Referee_Info.robot_status.chassis_power_limit>>8);
	SuperCap_TransmitPacket[7] = (Referee_Info.robot_status.chassis_power_limit);
	USER_CAN_TxMessage(CAN2,0x102,SuperCap_TransmitPacket,8);
	
	if(systick % 5 == 0)
	{
		Referee_Info_Report();
	}

    osDelayUntil(&systick,1);
  }
  /* USER CODE END CAN_Task */
}

static void Referee_Info_Report(void)
{
	static uint8_t shooter_type = 0;
	
	if(Referee_Info.robot_status.robot_level == 1)
	{
		if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 200)
		{
			/* Burst */
			shooter_type = 1;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 50)
		{
			/* Colling */
			shooter_type = 2;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 75)
		{
			/* rate */
			shooter_type = 3;
		}
	}
	else if(Referee_Info.robot_status.robot_level == 2)
	{
		if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 400)
		{
			/* Burst */
			shooter_type = 1;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 100)
		{
			/* Colling */
			shooter_type = 2;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 150)
		{
			/* rate */
			shooter_type = 3;
		}
	}
	else if(Referee_Info.robot_status.robot_level == 3)
	{
		if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 600)
		{
			/* Burst */
			shooter_type = 1;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 150)
		{
			/* Colling */
			shooter_type = 2;
		}
		else if(Referee_Info.robot_status.shooter_id1_17mm_cooling_limit >= 200)
		{
			/* rate */
			shooter_type = 3;
		}
	}
	
	MotorCAN_FramInfo[3][0] = (uint8_t)(Referee_Info.robot_status.robot_id >= 101) << 7 | \
														(uint8_t)(Referee_Info.robot_status.robot_level) << 5 | \
														(uint8_t)shooter_type << 2;
	
	MotorCAN_FramInfo[3][1] = (uint8_t)(Referee_Info.power_heat.shooter_id1_17mm_cooling_heat >> 8);
	MotorCAN_FramInfo[3][2] = (uint8_t)(Referee_Info.power_heat.shooter_id1_17mm_cooling_heat );
	MotorCAN_FramInfo[3][3] = (uint8_t)((int16_t)(Referee_Info.shoot_data.bullet_speed*100) >> 8);
	MotorCAN_FramInfo[3][4] = (uint8_t)((int16_t)(Referee_Info.shoot_data.bullet_speed*100) );
	
	USER_CAN_TxMessage(CAN2,0x300,MotorCAN_FramInfo[3],5);
}

