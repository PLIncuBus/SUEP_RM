#ifndef __AGV_CHASSIS_H
#define __AGV_CHASSIS_H

#include "chassis_task.h"
#include "remote_control.h"
#include "INS_task.h"

typedef struct
{
    chassis_motor_t motor_info;
    float Ecd_Now;
    float Ecd_Last;
    float Angle_Bias;
    float Angle_Target;
    float Angle_Set;
    float Angle_Now;
    float An
    int8_t turn_flg;
    pid_type_def pid;

}AGV_SteeringWheel_Info_Typedef;


typedef struct 
{
    chassis_motor_t motor_info;
    float Velocity_Target;
    int8_t direction;
    pid_type_def pid;

}AGV_PropulsionWheel_Info_Typedef;


typedef struct 
{

    float vx;
    float vy;
    float vz;

    float vx_max;
    float vx_min;
    float vy_max;
    float vy_min;

    float Sqrt[4][2];
    
}AGV_Mov_Info_Typedef;


typedef struct
{ 
    const fp32 *chassis_INS_angle; 
    const RC_ctrl_t *chassis_RC;
    
    AGV_SteeringWheel_Info_Typedef SteeringWheel[4];
    AGV_PropulsionWheel_Info_Typedef PropulsionWheel[4]; 
    AGV_Mov_Info_Typedef AGV_Mov;
    
}AGV_Handle_Typedef;






#endif