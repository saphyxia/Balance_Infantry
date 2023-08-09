/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/**
 * @note PI*0.51*100/(360.f), cm/s
 */
#define WHEEL_DPS_TO_VOLOCITY   0.44505895926f

/**
 * @note PI*0.085*100/360, cm
 */
#define MOMENTUM_ANGLE_TO_POSITION    0.07417649321f

/**
 * @note PI*0.085*100/(60*1), cm/s 
 */
#define MOMENTUM_RPM_TO_VOLOCITY    0.445058959258554f

/**
 * @note 2*PI/(60*1), rad/s 
 */
#define DJI_GM6020_RPM_TO_RADIANS   0.1047197551f

/**
 * @note 360/(8192*1), digree/s 
 */
#define DJI_GM6020_ENCODER_TO_DIGREES  0.0439453125f

/**
 * @brief typedef enum that contains the mode of chassis
*/
typedef enum 
{
  CHASSIS_WEAK,
  CHASSIS_FRONT,
  CHASSIS_SIDE,
  CHASSIS_SPIN,
  CHASSIS_SLIP,
  CHASSIS_MODE_NUM,
}Chassis_Mode_e;

/**
 * @brief typedef structure that contains the information of chassis control
*/
typedef struct 
{
  Chassis_Mode_e mode;

  int8_t Chassis_Direction;

  bool IF_YAW_ANGLE_OFFSET;

  bool IF_SuperCap_Eeable;

  struct
  {
    bool Init[2];
    int8_t Init_Direction;
    uint16_t Init_Cnt;
    float LeftInitial_Position[2];
    float RightInitial_Position[2];
    float Middle_Position[2];
  }Momentum;

  struct
  {
    float chassis_yawangle[2];
    float chassis_pitchangle;
    float chassis_velocity;
    float chassis_yawgyro;
    float chassis_position;
  }Target;
  
  struct 
  {
    float chassis_position;
    float chassis_yawangle;
    float chassis_pitchangle;
    float left_Momentum_position;
    float right_Momentum_position;
    float chassis_velocity;
    float chassis_yawgyro;
    float chassis_pitchgyro;
    float left_Momentum_velocity;
    float right_Momentum_velocity;
    float chassis_yawangle_err;
  }Measure;

  struct
  {
    int16_t chassis_velocity;
    int16_t chassis_position;
    int8_t wheel[2];
    int8_t momentum[2];
  }Limit;

  uint16_t Momentum_K;
  uint16_t Wheel_K;
	
	float power_k;

  float lqr_k[4][10];
  float lqr_out[4][2];
  float lqr_x[10];

  int16_t SendValue[2][2];

}Control_Info_Typedef;

extern Control_Info_Typedef Control_Info;

#endif //CONTROL_TASK_H
