#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include "chassis_task.h"
#include "remote_control.h"
#include "INS_task.h"


extern AGV_Handle_Typedef AGV_Handle;

typedef struct
{
    chassis_motor_t motor_info;
    float Ecd_Now;
    float Ecd_Last;
    float Angle_Bias;
    float Angle_Target;
    float Angle_Target_beforehand;
    float Angle_Target_Last;
    float Angle_Set;
    float Angle_Now;
    float Velocity_Now;
    float Round_Cnt_Now;
    float Round_Cnt_Target;
    pid_type_def InnerPid;
    pid_type_def OutPid;
    int8_t turn_flg;
}AGV_SteeringWheel_Info_Typedef;

typedef struct 
{
    chassis_motor_t motor_info;
    float Velocity_Target;
    float Velocity_Now;
    float Velocity_Set;
    int8_t direction;
    float v_max;
    float v_min;
    pid_type_def pid;
}AGV_PropulsionWheel_Info_Typedef;


typedef struct 
{

    float vx_set;
    float vy_set;
    float vz_set;

    float Sqrt[4][2];

    first_order_filter_type_t chassis_cmd_slow_set_vx; 
    first_order_filter_type_t chassis_cmd_slow_set_vy; 

}AGV_Mov_Info_Typedef;


typedef struct
{ 
    const fp32 *chassis_INS_angle; 
    const RC_ctrl_t *chassis_RC;
    
    AGV_SteeringWheel_Info_Typedef SteeringWheel[4];
    AGV_PropulsionWheel_Info_Typedef PropulsionWheel[4]; 
    AGV_Mov_Info_Typedef AGV_Mov;
    
}AGV_Handle_Typedef;


void AGV_Chassis_Init(AGV_Handle_Typedef *AGV_Chassis_Init);
void AGV_InverseKinematics(AGV_Handle_Typedef *AGV_InverseKinematics);
void AGV_SteerWheel_EcdToAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_EcdToAngle_Handle);
void AGV_RoundingToNearest_Handle(AGV_Handle_Typedef *AGV_RoundingToNearest_Handle);
void AGV_SteerWheel_TargetAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_TargetAngle_Handle);
void AGV_PID_Cal(AGV_Handle_Typedef *AGV_PID_Cal);
void AGV_Feedback_Update(AGV_Handle_Typedef *AGV_Feedback_Update);



#endif