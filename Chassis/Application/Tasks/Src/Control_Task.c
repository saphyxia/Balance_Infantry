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

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "ramp.h"
#include "pid.h"
#include "motor.h"
#include "arm_math.h"
#include "referee_info.h"
#include "bsp_dwt.h"
#include "bsp_can.h"

Control_Info_Typedef Control_Info = {
    .mode = CHASSIS_WEAK,
    .Momentum.Init = false,
    .Target.chassis_pitchangle = 0.f,    // rad
    .Target.chassis_yawangle[0] = 268.30f,    // digrees
    .Target.chassis_yawangle[1] = 1.3f,    // digrees
    .Limit.chassis_velocity = 500,     // cm
    .Limit.chassis_position = 100,      // cm
    .Limit.wheel = {10,4},
    .Limit.momentum = {30,3},
		.IF_SuperCap_Eeable = false,
    .lqr_k={
        [0]={  3.6487964849712,          38.5738847375239,          88.2000950305366,          7.05172886887703,   1.3818279276581,          12.9932999432037,          2.65808716841957,          17.4188486215712,  0.471551660945925,          0.34349785676495,},
        [1]={ 3.64879648497119,         -38.5738847375239,          88.2000950305367,          1.38182792765824,  7.05172886887693,          12.9932999432037,         -2.65808716841956,           17.418848621571,   0.34349785676497,         0.471551660945923,},
        [2]={-1.82842612802097,          2.45513912601843,         -8.25655136377268,          -45.263447479975,-0.722162006176724,         -6.37909471730117,         0.485294042460812,         -0.77539166342401,  -6.81355714229407,       -0.0726595773048069,},
        [3]={-1.82842612802099,         -2.45513912601843,         -8.25655136377333,        -0.722162006176728, -45.2634474799747,         -6.37909471730123,        -0.485294042460814,        -0.775391663423992,-0.0726595773048198,         -6.81355714229405,},
    },
};   
         
PID_Info_TypeDef Momentum_Init_Pid[2];
float Momentum_Init_Pid_Param[PID_PARAMETER_NUM]={100,0,0,0,0,8000};

uint32_t Control_DWT_TIME;

float Control_DWT_DELTA;

static Chassis_Mode_e Chassis_Mode_Previous;

static void Control_Mode_Update(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info);
static void Control_CAN_Update(Control_Info_Typedef *Control_Info);

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t systick = 0;

  PID_Init(&Momentum_Init_Pid[0],PID_POSITION,Momentum_Init_Pid_Param);
  PID_Init(&Momentum_Init_Pid[1],PID_POSITION,Momentum_Init_Pid_Param);

  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();

    Control_DWT_DELTA = DWT_GetDeltaT(&Control_DWT_TIME);

    Control_Mode_Update(&Control_Info);

    Control_Measure_Update(&Control_Info);

    if(Control_Info.Momentum.Init[0]==true && Control_Info.Momentum.Init[1]==true )
    {
        Control_LQR_X_Update(&Control_Info);
        Control_CAN_Update(&Control_Info);
    }
    else
    {
        Control_Info.SendValue[0][0] = 0;
        Control_Info.SendValue[0][1] = 0; 
    }
	 
    osDelayUntil(&systick,1);
  }
  /* USER CODE END Control_Task */
}


static void Control_Mode_Update(Control_Info_Typedef *Control_Info)
{
		if(remote_ctrl.rc.s[0] == 3)
    {
        if(Key_Ctrl()==true)
        {
            Control_Info->mode = CHASSIS_SIDE;
        }
        else
        {
            Control_Info->mode = CHASSIS_FRONT;
        }
    }
    else if(remote_ctrl.rc.s[0] == 2)
    {
        Control_Info->mode = CHASSIS_SIDE;
    }
		else
		{
				Control_Info->mode = CHASSIS_WEAK;
		}

    if((Key_Shift()==true || remote_ctrl.rc.ch[4]==1) && Control_Info->mode != CHASSIS_WEAK)
    {
        Control_Info->mode = CHASSIS_SPIN;
    }

    if((Key_F()==true || remote_ctrl.rc.ch[4]==-1))
    {
        Control_Info->mode = CHASSIS_SLIP;
    }

    if(Control_Info->Momentum.Init[0] == false || Control_Info->Momentum.Init[1] == false)
    {
        Control_Info->mode = CHASSIS_WEAK;
    }
		
		if(Chassis_Mode_Previous != CHASSIS_WEAK && Control_Info->mode == CHASSIS_WEAK)
		{
			Control_Info->Momentum.Init[0] = false;
			Control_Info->Momentum.Init[1] = false;
		}

		Chassis_Mode_Previous = Control_Info->mode;
}

static void Momentum_Position_Init(Control_Info_Typedef *Control_Info)
{
	if(Control_Info->Momentum.Init[0] != true)
    {
        Control_Info->SendValue[1][0] = f_PID_Calculate(&Momentum_Init_Pid[0],-Control_Info->Momentum.Init_Direction*200,DJI_Motor[Left_Momentum].Data.velocity);
        Control_Info->SendValue[1][1] = f_PID_Calculate(&Momentum_Init_Pid[1], Control_Info->Momentum.Init_Direction*200,DJI_Motor[Right_Momentum].Data.velocity);
    }
    else
    {
        Control_Info->SendValue[1][0] = f_PID_Calculate(&Momentum_Init_Pid[0], Control_Info->Momentum.Init_Direction*200,DJI_Motor[Left_Momentum].Data.velocity);
        Control_Info->SendValue[1][1] = f_PID_Calculate(&Momentum_Init_Pid[1],-Control_Info->Momentum.Init_Direction*200,DJI_Motor[Right_Momentum].Data.velocity);
    }
	
	if(DJI_Motor[Left_Momentum].Data.last_encoder == DJI_Motor[Left_Momentum].Data.encoder && DJI_Motor[Right_Momentum].Data.last_encoder == DJI_Motor[Right_Momentum].Data.encoder)
	{
        Control_Info->Momentum.Init_Cnt++;
	}
	else
	{
        Control_Info->Momentum.Init_Cnt = 0;
	}
	
	if(Control_Info->Momentum.Init_Cnt >= 0x3E8U)
	{
		if(Control_Info->Momentum.Init[0] != true)
		{
				Control_Info->Momentum.Init[0] = true;
				Control_Info->Momentum.Init_Cnt = 0;
				Control_Info->Momentum.LeftInitial_Position[0] = DJI_Motor[Left_Momentum].Data.angle;
				Control_Info->Momentum.RightInitial_Position[0] = DJI_Motor[Right_Momentum].Data.angle;
		}
		else
		{
				Control_Info->Momentum.Init[1] = true;
				Control_Info->Momentum.Init_Cnt = 0;
				Control_Info->Momentum.LeftInitial_Position[1] = DJI_Motor[Left_Momentum].Data.angle;
				Control_Info->Momentum.RightInitial_Position[1] = DJI_Motor[Right_Momentum].Data.angle;

				Control_Info->Momentum.Middle_Position[0] = 0.5f*fabsf(Control_Info->Momentum.LeftInitial_Position[0]-Control_Info->Momentum.LeftInitial_Position[1]);
				Control_Info->Momentum.Middle_Position[1] = 0.5f*fabsf(Control_Info->Momentum.RightInitial_Position[0]-Control_Info->Momentum.RightInitial_Position[1]);
		}
		Control_Info->SendValue[1][0] = 0;
		Control_Info->SendValue[1][1] = 0;
	}
}

static void Momentum_Position_Update(Control_Info_Typedef *Control_Info,float left_angle,float right_angle)
{
	if(Control_Info->Momentum.Init[0] != true || Control_Info->Momentum.Init[1] != true)
	{
		if(INS_Info.pit_angle < 0)
		{
				Control_Info->Momentum.Init_Direction = 1;
		}
		else if(INS_Info.pit_angle > 0)
		{
				Control_Info->Momentum.Init_Direction = -1;
		}

		Momentum_Position_Init(Control_Info);
	}
	Control_Info->Measure.left_Momentum_position   = -((left_angle - Control_Info->Momentum.LeftInitial_Position[0]) -Control_Info->Momentum.Init_Direction*Control_Info->Momentum.Middle_Position[0])*MOMENTUM_ANGLE_TO_POSITION;
	Control_Info->Measure.right_Momentum_position  =  ((right_angle- Control_Info->Momentum.RightInitial_Position[0])+Control_Info->Momentum.Init_Direction*Control_Info->Momentum.Middle_Position[1])*MOMENTUM_ANGLE_TO_POSITION;
}

static float Yaw_Error_Update(float error,float halfmax)
{
	float result = 0.f;
	
	if(fabsf(error) > halfmax)
        result =  error - error/fabsf(error)*halfmax*2.f;
	else
        result = error;
	
	return result;
}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info)
{	
		if(Control_Info->mode == CHASSIS_SIDE)
		{
			Control_Info->IF_YAW_ANGLE_OFFSET = true;
		}
		else
		{
			Control_Info->IF_YAW_ANGLE_OFFSET = false;
		}

		if(fabsf(Control_Info->Measure.chassis_yawangle - 268.30f) <= fabsf(Control_Info->Measure.chassis_yawangle - 88.30f))
		{
        Control_Info->Chassis_Direction = 1;
        Control_Info->Target.chassis_yawangle[0] = 268.30f;
    }
    else
    {
        Control_Info->Chassis_Direction = -1;
        Control_Info->Target.chassis_yawangle[0] = 88.30f;
    }

    if(remote_ctrl.rc.ch[3] != 0)
    {
        Control_Info->Target.chassis_velocity = f_Ramp_Calc(Control_Info->Target.chassis_velocity,-Control_Info->Chassis_Direction*remote_ctrl.rc.ch[3],1.f);
        VAL_LIMIT(Control_Info->Target.chassis_velocity,-Control_Info->Limit.chassis_velocity,Control_Info->Limit.chassis_velocity);
    }
    else if(Key_W() != 0 || Key_S() != 0)
    {
        Control_Info->Target.chassis_velocity = f_Ramp_Calc(Control_Info->Target.chassis_velocity,-Control_Info->Chassis_Direction*(Key_W()-Key_S())*Control_Info->Limit.chassis_velocity,0.2f);
        VAL_LIMIT(Control_Info->Target.chassis_velocity,-Control_Info->Limit.chassis_velocity,Control_Info->Limit.chassis_velocity);
    }
    else
    {
        Control_Info->Target.chassis_velocity = f_Ramp_Calc(Control_Info->Target.chassis_velocity,0,0.2f);
        VAL_LIMIT(Control_Info->Target.chassis_velocity,-Control_Info->Limit.chassis_velocity,Control_Info->Limit.chassis_velocity);
    }

    if(Control_Info->mode == CHASSIS_SPIN)
    {
        Control_Info->Target.chassis_yawgyro = 8;
    }
    else
    {
        Control_Info->Target.chassis_yawgyro = 0;
    }

//    if(Control_Info->Target.chassis_velocity == 0 )
//    {
				Control_Info->Target.chassis_position = 0;
        Control_Info->Measure.chassis_position -= Control_Info->Measure.chassis_velocity*Control_DWT_DELTA;
				if(fabsf(Control_Info->Measure.chassis_position)>Control_Info->Limit.chassis_position)
					Control_Info->Measure.chassis_position = 0;
//    }
//    else
//    {
//        Control_Info->Measure.chassis_position = 0;
//    }

    Control_Info->Measure.chassis_yawangle = DJI_Motor[Yaw].Data.encoder*DJI_GM6020_ENCODER_TO_DIGREES;
    Control_Info->Measure.chassis_yawangle_err = Yaw_Error_Update(Control_Info->Measure.chassis_yawangle - Control_Info->Target.chassis_yawangle[Control_Info->IF_YAW_ANGLE_OFFSET],180)*DegreesToRadians;
    Control_Info->Measure.chassis_pitchangle = INS_Info.angle[IMU_ANGLE_INDEX_PITCH];

    Momentum_Position_Update(Control_Info,DJI_Motor[Left_Momentum].Data.angle,DJI_Motor[Right_Momentum].Data.angle);

    Control_Info->Measure.chassis_velocity = (float)(-RMD_Motor[Left_Wheel].Data.velocity + RMD_Motor[Right_Wheel].Data.velocity)/2.f*WHEEL_DPS_TO_VOLOCITY - Control_Info->Measure.chassis_pitchgyro*0.105f;
    Control_Info->Measure.chassis_yawgyro = (float)(RMD_Motor[Left_Wheel].Data.velocity + RMD_Motor[Right_Wheel].Data.velocity)/2.f*WHEEL_DPS_TO_VOLOCITY/51.f;
    Control_Info->Measure.chassis_pitchgyro = INS_Info.gyro[IMU_GYRO_INDEX_PITCH]*arm_cos_f32(INS_Info.angle[IMU_ANGLE_INDEX_ROLL]) - INS_Info.gyro[IMU_GYRO_INDEX_YAW]*arm_sin_f32(INS_Info.angle[IMU_ANGLE_INDEX_ROLL]);
    Control_Info->Measure.left_Momentum_velocity = -DJI_Motor[Left_Momentum].Data.velocity*MOMENTUM_RPM_TO_VOLOCITY;
    Control_Info->Measure.right_Momentum_velocity = DJI_Motor[Right_Momentum].Data.velocity*MOMENTUM_RPM_TO_VOLOCITY;
}

static void Control_LQR_X_Update(Control_Info_Typedef *Control_Info)
{
    Control_Info->power_k = Referee_Info.power_heat.chassis_power_buffer/60.f;
    VAL_LIMIT(Control_Info->power_k,0,1.f);

    if(Control_Info->power_k > 0.4f)
    {
        Control_Info->power_k = sqrt(Control_Info->power_k);
    }

		if(SuperCap_ReceivePacket.ele_quantity <= 20 || Key_Q() == false)
		{
			Control_Info->IF_SuperCap_Eeable = false;
		}
		else
		{
			Control_Info->IF_SuperCap_Eeable = true;
		}
		
    Control_Info->lqr_x[0] = (Control_Info->Target.chassis_position - Control_Info->Measure.chassis_position)/100.f;
    Control_Info->lqr_x[1] = (Control_Info->Measure.chassis_yawangle_err);
    Control_Info->lqr_x[2] = (Control_Info->Target.chassis_pitchangle - Control_Info->Measure.chassis_pitchangle);
    Control_Info->lqr_x[3] = (0 - Control_Info->Measure.left_Momentum_position)/100.f;
    Control_Info->lqr_x[4] = (0 - Control_Info->Measure.right_Momentum_position)/100.f;
    Control_Info->lqr_x[5] = (Control_Info->Target.chassis_velocity - Control_Info->Measure.chassis_velocity)/100.f*f_LogisticCurves_Calc(fabsf(INS_Info.pit_angle),0.6f,38.f);;
    Control_Info->lqr_x[6] = (Control_Info->Target.chassis_yawgyro - Control_Info->Measure.chassis_yawgyro);
    Control_Info->lqr_x[7] = (0 - Control_Info->Measure.chassis_pitchgyro);
    Control_Info->lqr_x[8] = (0-Control_Info->Measure.left_Momentum_velocity)/100.f;
    Control_Info->lqr_x[9] = (0-Control_Info->Measure.right_Momentum_velocity)/100.f;

    if(Control_Info->mode == CHASSIS_SPIN)
    {
        Control_Info->lqr_x[1] = 0;
    }

    /* wheel ------------------------------------------------------------------*/
    Control_Info->lqr_out[0][0] =-Control_Info->lqr_k[0][0]*Control_Info->lqr_x[0] + \
                                 -Control_Info->lqr_k[0][2]*Control_Info->lqr_x[2] + \
                                 -Control_Info->lqr_k[0][3]*Control_Info->lqr_x[3] + \
                                 -Control_Info->lqr_k[0][4]*Control_Info->lqr_x[4] + \
                                  Control_Info->lqr_k[0][5]*Control_Info->lqr_x[5] + \
                                 -Control_Info->lqr_k[0][7]*Control_Info->lqr_x[7] + \
                                  Control_Info->lqr_k[0][8]*Control_Info->lqr_x[8] + \
                                  Control_Info->lqr_k[0][9]*Control_Info->lqr_x[9];
    VAL_LIMIT(Control_Info->lqr_out[0][0],-Control_Info->Limit.wheel[0],Control_Info->Limit.wheel[0]);

    Control_Info->lqr_out[0][1] = Control_Info->lqr_k[0][1]*Control_Info->lqr_x[1] + \
                                  Control_Info->lqr_k[0][6]*Control_Info->lqr_x[6]; 
    VAL_LIMIT(Control_Info->lqr_out[0][1],-Control_Info->Limit.wheel[1],Control_Info->Limit.wheel[1]);

    Control_Info->lqr_out[1][0] = Control_Info->lqr_k[1][0]*Control_Info->lqr_x[0] + \
                                  Control_Info->lqr_k[1][2]*Control_Info->lqr_x[2] + \
                                  Control_Info->lqr_k[1][3]*Control_Info->lqr_x[3] + \
                                  Control_Info->lqr_k[1][4]*Control_Info->lqr_x[4] + \
                                 -Control_Info->lqr_k[1][5]*Control_Info->lqr_x[5] + \
                                  Control_Info->lqr_k[1][7]*Control_Info->lqr_x[7] + \
                                 -Control_Info->lqr_k[1][8]*Control_Info->lqr_x[8] + \
                                 -Control_Info->lqr_k[1][9]*Control_Info->lqr_x[9];
    VAL_LIMIT(Control_Info->lqr_out[1][0],-Control_Info->Limit.wheel[0],Control_Info->Limit.wheel[0]);

    Control_Info->lqr_out[1][1] =-Control_Info->lqr_k[1][1]*Control_Info->lqr_x[1] + \
                                 -Control_Info->lqr_k[1][6]*Control_Info->lqr_x[6]; 
    VAL_LIMIT(Control_Info->lqr_out[1][1],-Control_Info->Limit.wheel[1],Control_Info->Limit.wheel[1]);

    /* Memontum ------------------------------------------------------------------*/
    Control_Info->lqr_out[2][0] = Control_Info->lqr_k[2][0]*Control_Info->lqr_x[0] + \
                                  Control_Info->lqr_k[2][2]*Control_Info->lqr_x[2] + \
                                  Control_Info->lqr_k[2][3]*Control_Info->lqr_x[3] + \
                                  Control_Info->lqr_k[2][4]*Control_Info->lqr_x[4] + \
                                 -Control_Info->lqr_k[2][5]*Control_Info->lqr_x[5] + \
                                  Control_Info->lqr_k[2][7]*Control_Info->lqr_x[7] + \
																 -Control_Info->lqr_k[2][8]*Control_Info->lqr_x[8] + \
																 -Control_Info->lqr_k[2][9]*Control_Info->lqr_x[9];
    VAL_LIMIT(Control_Info->lqr_out[2][0],-Control_Info->Limit.momentum[0],Control_Info->Limit.momentum[0]);

    Control_Info->lqr_out[2][1] =-Control_Info->lqr_k[2][1]*Control_Info->lqr_x[1] + \
                                  Control_Info->lqr_k[2][6]*Control_Info->lqr_x[6];   
    VAL_LIMIT(Control_Info->lqr_out[2][1],-Control_Info->Limit.momentum[1],Control_Info->Limit.momentum[1]);

    Control_Info->lqr_out[3][0] =-Control_Info->lqr_k[3][0]*Control_Info->lqr_x[0] + \
                                 -Control_Info->lqr_k[3][2]*Control_Info->lqr_x[2] + \
                                 -Control_Info->lqr_k[3][3]*Control_Info->lqr_x[3] + \
                                 -Control_Info->lqr_k[3][4]*Control_Info->lqr_x[4] + \
                                  Control_Info->lqr_k[3][5]*Control_Info->lqr_x[5] + \
                                 -Control_Info->lqr_k[3][7]*Control_Info->lqr_x[7] + \
                                  Control_Info->lqr_k[3][8]*Control_Info->lqr_x[8] + \
                                  Control_Info->lqr_k[3][9]*Control_Info->lqr_x[9];
    VAL_LIMIT(Control_Info->lqr_out[3][0],-Control_Info->Limit.momentum[0],Control_Info->Limit.momentum[0]);

    Control_Info->lqr_out[3][1] = Control_Info->lqr_k[3][1]*Control_Info->lqr_x[1] + \
                                 -Control_Info->lqr_k[3][6]*Control_Info->lqr_x[6];   
    VAL_LIMIT(Control_Info->lqr_out[3][1],-Control_Info->Limit.momentum[1],Control_Info->Limit.momentum[1]);
}

static void Control_CAN_Update(Control_Info_Typedef *Control_Info)
{
    if(Control_Info->mode == CHASSIS_SLIP)
    {
        Control_Info->SendValue[0][0] = ((-Control_Info->Target.chassis_velocity - RMD_Motor[Left_Wheel].Data.velocity*WHEEL_DPS_TO_VOLOCITY )*2.f+ Control_Info->lqr_out[0][1]*100)*Control_Info->power_k;
        Control_Info->SendValue[0][1] = (( Control_Info->Target.chassis_velocity - RMD_Motor[Right_Wheel].Data.velocity*WHEEL_DPS_TO_VOLOCITY)*2.f+ Control_Info->lqr_out[0][1]*100)*Control_Info->power_k;
    }
    else if(Control_Info->mode != CHASSIS_WEAK)
    {
        Control_Info->SendValue[0][0] = (Control_Info->lqr_out[0][0] + Control_Info->lqr_out[0][1] )*100*Control_Info->power_k;
        Control_Info->SendValue[0][1] = (Control_Info->lqr_out[1][0] + Control_Info->lqr_out[1][1] )*100*Control_Info->power_k;

        Control_Info->SendValue[1][0] = (Control_Info->lqr_out[2][0] + Control_Info->lqr_out[2][1] )*500;
        Control_Info->SendValue[1][1] = (Control_Info->lqr_out[3][0] + Control_Info->lqr_out[3][1] )*500;
				
				if(Control_Info->Measure.left_Momentum_position == Control_Info->Momentum.LeftInitial_Position[0] || Control_Info->Measure.left_Momentum_position == Control_Info->Momentum.LeftInitial_Position[1])
				{
					Control_Info->SendValue[1][0] = 0;
				}
				
				if(Control_Info->Measure.right_Momentum_position == Control_Info->Momentum.RightInitial_Position[0] || Control_Info->Measure.right_Momentum_position == Control_Info->Momentum.RightInitial_Position[1])
				{
					Control_Info->SendValue[1][1] = 0;
				}
    }
    else
    {
        Control_Info->SendValue[0][0] = 0;
        Control_Info->SendValue[0][1] = 0;

        Control_Info->SendValue[1][0] = 0;
        Control_Info->SendValue[1][1] = 0;
    }
		
}


