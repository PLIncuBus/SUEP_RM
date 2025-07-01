#include "AGV_Chassis.h"
#include "AGV_Chassis_param.h"
#include "arm_math.h"


void AGV_Feedback_Update(AGV_Handle_Typedef *AGV_Feeback_Update);

/**
  * @brief          AGV底盘初始化
  * @param[out]     //AGV_Handle_Typedef *AGV_Chassis_Init
  * @retval         none
  */
void AGV_Chassis_Init(AGV_Handle_Typedef *AGV_Chassis_Init)
{
    const float AGV_SteeringWheel_innner_PID[3] = {AGV_SteeringWheel_innner_Kp,AGV_SteeringWheel_innner_Ki,AGV_SteeringWheel_innner_Kd};
    const float AGV_SteeringWheel_out_PID[3]    = {AGV_SteeringWheel_out_Kp,AGV_SteeringWheel_out_Ki,AGV_SteeringWheel_out_Kd};
    const float AGV_PropulsionWheel_PID[3]      = {AGV_PropulsionWheel_Kp,AGV_PropulsionWheel_Ki,AGV_PropulsionWheel_Kd};

    //初始化舵向角度偏置
    AGV_Chassis_Init->SteeringWheel[0].Angle_Bias = AGV_Chassis_SteeringWheel1_Bias;
    AGV_Chassis_Init->SteeringWheel[1].Angle_Bias = AGV_Chassis_SteeringWheel2_Bias;
    AGV_Chassis_Init->SteeringWheel[2].Angle_Bias = AGV_Chassis_SteeringWheel3_Bias;
    AGV_Chassis_Init->SteeringWheel[3].Angle_Bias = AGV_Chassis_SteeringWheel4_Bias;

    AGV_Chassis_Init->PropulsionWheel[0].direction = -1;
    AGV_Chassis_Init->PropulsionWheel[1].direction = -1;
    AGV_Chassis_Init->PropulsionWheel[2].direction = 1;
    AGV_Chassis_Init->PropulsionWheel[3].direction = 1;
    

    AGV_Chassis_Init->chassis_RC = get_remote_control_point();
    AGV_Chassis_Init->chassis_INS_angle = get_INS_angle_point();

    
    for(uint8_t i = 0;i < 4;i ++)
    {    
        //获取电机状态指针
        AGV_Chassis_Init->PropulsionWheel[i].motor_info.chassis_motor_measure = get_chassis_motor_measure_point(i);
        AGV_Chassis_Init->SteeringWheel[i].motor_info.chassis_motor_measure   = get_chassis_motor_measure_point(i + 4);

        //PID初始化，速度内环+角度外环
        PID_init(&AGV_Chassis_Init->SteeringWheel[i].pid,PID_DELTA,AGV_SteeringWheel_innner_PID,AGV_SteeringWheel_inner_max_out,AGV_SteeringWheel_inner_min_out);
        PID_init(&AGV_Chassis_Init->SteeringWheel[i].pid,PID_POSITION,AGV_SteeringWheel_out_PID,AGV_SteeringWheel_out_max_out,AGV_SteeringWheel_out_min_out);
        PID_init(&AGV_Chassis_Init->PropulsionWheel[i].pid,PID_DELTA,AGV_PropulsionWheel_PID,AGV_PropulsionWheel_max_out,AGV_PropulsionWheel_min_out);
    }  

    AGV_Chassis_Init->AGV_Mov.vx_max = AGV_MAX_VX;
    AGV_Chassis_Init->AGV_Mov.vx_min = -AGV_MAX_VX;

    AGV_Chassis_Init->AGV_Mov.vy_max = AGV_MAX_VY;
    AGV_Chassis_Init->AGV_Mov.vy_min = -AGV_MAX_VY;

    AGV_Feedback_Update(AGV_Chassis_Init);
}


/**
  * @brief          AGV运动学逆解(由Vx\Vy\Vz--->Angle/Linear)，获得Angle_Target参数
  * @param[out]     //AGV_Handle_Typedef *AGV_InverseKinematics
  * @retval         none
  * @note           2------3
  *                 |      |
  *                 1------0
  */
static void AGV_InverseKinematics(AGV_Handle_Typedef *AGV_InverseKinematics)
{
    //角度逆解算
    AGV_InverseKinematics->SteeringWheel[0].Angle_Target = atan2((AGV_InverseKinematics->AGV_Mov.vy-AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx-AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[1].Angle_Target = atan2((AGV_InverseKinematics->AGV_Mov.vy-AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx+AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[2].Angle_Target = atan2((AGV_InverseKinematics->AGV_Mov.vy+AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx+AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[3].Angle_Target = atan2((AGV_InverseKinematics->AGV_Mov.vy+AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx-AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f))); 
    //线速度逆结算    
    AGV_InverseKinematics->AGV_Mov.Sqrt[0][0] =  AGV_InverseKinematics->AGV_Mov.vy - AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[0][1] =  AGV_InverseKinematics->AGV_Mov.vx - AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[0][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[0][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][1],&AGV_InverseKinematics->PropulsionWheel[0].Velocity_Target)*M3508_MOTOR_RPM_TO_VECTOR*AGV_InverseKinematics->PropulsionWheel[0].direction;

    AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] =  AGV_InverseKinematics->AGV_Mov.vy - AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[1][1] =  AGV_InverseKinematics->AGV_Mov.vx + AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[1][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[1][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][1],&AGV_InverseKinematics->PropulsionWheel[1].Velocity_Target)*M3508_MOTOR_RPM_TO_VECTOR*AGV_InverseKinematics->PropulsionWheel[1].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] =  AGV_InverseKinematics->AGV_Mov.vy + AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][1] =  AGV_InverseKinematics->AGV_Mov.vx + AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[2][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[2][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][1],&AGV_InverseKinematics->PropulsionWheel[2].Velocity_Target)*M3508_MOTOR_RPM_TO_VECTOR*AGV_InverseKinematics->PropulsionWheel[2].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] =  AGV_InverseKinematics->AGV_Mov.vy + AGV_InverseKinematics->AGV_Mov.vz*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][1] =  AGV_InverseKinematics->AGV_Mov.vx - AGV_InverseKinematics->AGV_Mov.vz*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[3][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[3][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][1],&AGV_InverseKinematics->PropulsionWheel[3].Velocity_Target)*M3508_MOTOR_RPM_TO_VECTOR*AGV_InverseKinematics->PropulsionWheel[3].direction;  
}

/**
  * @brief          AGV舵向电机编码器转角度处理
  * @param[out]     //AGV_Handle_Typedef *AGV_SteerWheel_EcdToAngle_Handle
  * @retval         none
  * @note           因为回传的是增量式角度，所以需要做处理，使用的int32_t,无须担心堆溢出的情况
  */
static void AGV_SteerWheel_EcdToAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_EcdToAngle_Handle)
{
    AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel.Ecd_Now = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel.motor_info.chassis_motor_measure->ecd / 8192 * 360;
    if(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel->Ecd_Now - AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel->Ecd_Last > 180)
    {
        
    }
    AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel->Ecd_Last = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel->Ecd_Now; 

}
/**
  * @brief          AGV舵向电机目标角度处理
  * @param[out]     //AGV_Handle_Typedef *AGV_SteerWheel_TargetAngle_Handle
  * @retval         none
  * @note           +初始偏置以及就近原则处理
  * @note           因为目标输出是增量式角度，所以需要做处理，使用的int32_t,无须担心堆溢出的情况
  */
static void AGV_SteerWheel_TargetAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_TargetAngle_Handle)
{

    for(uint8_t i = 0;i < 4;i ++)
    {
        if(AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].turn_flg == 1)
        {
            AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target =  AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Set +
        }
    }


}

/**
  * @brief          AGV参数更新
  * @param[out]     AGV_Handle_Typedef *AGV_Feeback_Update
  * @retval         none
  */
void AGV_Feedback_Update(AGV_Handle_Typedef *AGV_Feeback_Update)
{

    //运动学逆结算
    AGV_InverseKinematics(AGV_Feedback_Update);


}

