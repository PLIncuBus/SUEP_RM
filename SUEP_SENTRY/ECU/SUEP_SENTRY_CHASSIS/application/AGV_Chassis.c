#include "AGV_Chassis.h"
#include "AGV_Chassis_param.h"
#include "arm_math.h"
#include "chassis_task.h"
#include "user_lib.h"



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

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    
    //初始化舵向角度偏置
    AGV_Chassis_Init->SteeringWheel[0].Angle_Bias = AGV_Chassis_SteeringWheel1_Bias;
    AGV_Chassis_Init->SteeringWheel[1].Angle_Bias = AGV_Chassis_SteeringWheel2_Bias;
    AGV_Chassis_Init->SteeringWheel[2].Angle_Bias = AGV_Chassis_SteeringWheel3_Bias;
    AGV_Chassis_Init->SteeringWheel[3].Angle_Bias = AGV_Chassis_SteeringWheel4_Bias;

    AGV_Chassis_Init->PropulsionWheel[0].direction = -1;
    AGV_Chassis_Init->PropulsionWheel[1].direction = -1;
    AGV_Chassis_Init->PropulsionWheel[2].direction = 1;
    AGV_Chassis_Init->PropulsionWheel[3].direction = 1;

    AGV_Chassis_Init->SteeringWheel[0].turn_flg = 0;
    AGV_Chassis_Init->SteeringWheel[1].turn_flg = 0;
    AGV_Chassis_Init->SteeringWheel[2].turn_flg = 0;
    AGV_Chassis_Init->SteeringWheel[3].turn_flg = 0;

    AGV_Chassis_Init->chassis_RC = get_remote_control_point();
    AGV_Chassis_Init->chassis_INS_angle = get_INS_angle_point();

    first_order_filter_init(&AGV_Chassis_Init->AGV_Mov.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&AGV_Chassis_Init->AGV_Mov.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    
    for(uint8_t i = 0;i < 4;i ++)
    {    
        //获取电机状态指针
        AGV_Chassis_Init->PropulsionWheel[i].motor_info.chassis_motor_measure = get_chassis_motor_measure_point(i);
        AGV_Chassis_Init->SteeringWheel[i].motor_info.chassis_motor_measure   = get_chassis_motor_measure_point(i + 4);

        //PID初始化，速度内环+角度外环
        PID_init(&AGV_Chassis_Init->SteeringWheel[i].InnerPid,PID_DELTA,AGV_SteeringWheel_innner_PID,AGV_SteeringWheel_inner_max_out,AGV_SteeringWheel_inner_min_out);
        PID_init(&AGV_Chassis_Init->SteeringWheel[i].OutPid,PID_POSITION,AGV_SteeringWheel_out_PID,AGV_SteeringWheel_out_max_out,AGV_SteeringWheel_out_min_out);
        PID_init(&AGV_Chassis_Init->PropulsionWheel[i].pid,PID_DELTA,AGV_PropulsionWheel_PID,AGV_PropulsionWheel_max_out,AGV_PropulsionWheel_min_out);
      

        AGV_Chassis_Init->PropulsionWheel[i].v_max = AGV_MAX_VX;
        AGV_Chassis_Init->PropulsionWheel[i].v_min = -AGV_MAX_VX;
    }



}

/**
  * @brief          AGV反馈参数更新
  * @param[out]     AGV_Handle_Typedef *AGV_Feeback_Update
  * @retval         none
  */
void AGV_Feedback_Update(AGV_Handle_Typedef *AGV_Feedback_Update)
{
  for(uint8_t i = 0; i < 4;i ++)
  {
    AGV_Feedback_Update->PropulsionWheel[i].Velocity_Now =   AGV_Feedback_Update->PropulsionWheel[i].motor_info.chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
    AGV_Feedback_Update->SteeringWheel[i].Velocity_Now   =   AGV_Feedback_Update->SteeringWheel[i].motor_info.chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
  }
}

/**
  * @brief          AGV运动学逆解(由Vx\Vy\Vz--->Angle/Linear)，获得Angle_Target参数
  * @param[out]     //AGV_Handle_Typedef *AGV_InverseKinematics
  * @retval         none
  * @note           2------3
  *                 |      |
  *                 1------0
  */
void AGV_InverseKinematics(AGV_Handle_Typedef *AGV_InverseKinematics)
{
    //角度逆解算
    AGV_InverseKinematics->SteeringWheel[0].Angle_Set = atan2((AGV_InverseKinematics->AGV_Mov.vy_set-AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx_set-AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[1].Angle_Set = atan2((AGV_InverseKinematics->AGV_Mov.vy_set-AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx_set+AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[2].Angle_Set = atan2((AGV_InverseKinematics->AGV_Mov.vy_set+AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx_set+AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)));
    AGV_InverseKinematics->SteeringWheel[3].Angle_Set = atan2((AGV_InverseKinematics->AGV_Mov.vy_set+AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)),
                                                                   (AGV_InverseKinematics->AGV_Mov.vx_set-AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f))); 
    //线速度逆结算    
    AGV_InverseKinematics->AGV_Mov.Sqrt[0][0] =  AGV_InverseKinematics->AGV_Mov.vy_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[0][1] =  AGV_InverseKinematics->AGV_Mov.vx_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[0][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[0][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][1],&AGV_InverseKinematics->PropulsionWheel[0].Velocity_Set)*AGV_InverseKinematics->PropulsionWheel[0].direction;

    AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] =  AGV_InverseKinematics->AGV_Mov.vy_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[1][1] =  AGV_InverseKinematics->AGV_Mov.vx_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[1][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[1][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][1],&AGV_InverseKinematics->PropulsionWheel[1].Velocity_Set)*AGV_InverseKinematics->PropulsionWheel[1].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] =  AGV_InverseKinematics->AGV_Mov.vy_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][1] =  AGV_InverseKinematics->AGV_Mov.vx_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[2][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[2][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][1],&AGV_InverseKinematics->PropulsionWheel[2].Velocity_Set)*AGV_InverseKinematics->PropulsionWheel[2].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] =  AGV_InverseKinematics->AGV_Mov.vy_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][1] =  AGV_InverseKinematics->AGV_Mov.vx_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[3][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[3][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][1],&AGV_InverseKinematics->PropulsionWheel[3].Velocity_Set)*AGV_InverseKinematics->PropulsionWheel[3].direction;  
}

/**
  * @brief          AGV舵向电机编码器转角度处理
  * @param[out]     //AGV_Handle_Typedef *AGV_SteerWheel_EcdToAngle_Handle
  * @retval         none
  * @note           当前角度 = 当前角度值 + 增量式 +初始偏置
  * @note           因为回传的是增量式角度，所以需要做处理，使用的float(-2^23-2^23),无须担心补码的情况
  */
void AGV_SteerWheel_EcdToAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_EcdToAngle_Handle)
{
  for(uint8_t i = 0;i < 4;i ++ )
  {
      AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].motor_info.chassis_motor_measure->ecd / 8192 * 360;
      if(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now - AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Last > 180)
      {
        AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now --;
      }
      else if(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now - AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Last < -180)
      {
        AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now ++;
      }
      AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Angle_Now = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now*360 +AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now + AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Angle_Bias;
      AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Last = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now; 
  }
}

/**
  * @brief          AGV就近原则处理
  * @param[out]     //AGV_Handle_Typedef *AGV_RoundingToNearest
  * @retval         none
  */
void AGV_RoundingToNearest_Handle(AGV_Handle_Typedef *AGV_RoundingToNearest_Handle)
{
  for(uint8_t i = 0;i < 4;i ++)
  {
      if(AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_beforehand - AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_Last > 180)
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg = 1;
      }
      else if(AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_beforehand - AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_Last < -180)
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg = -1;
      }
      else 
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg = 0;
      }


      if(AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg == 1)
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target =  AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_beforehand - 360;
        AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Target = - AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Set;

      }
      else if(AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg == -1)
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target =  AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_beforehand + 360;
        AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Target = - AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Set;
      }
      else if(AGV_RoundingToNearest_Handle->SteeringWheel[i].turn_flg == 0)
      {
        AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target =  AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_beforehand;
        AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Target = AGV_RoundingToNearest_Handle->PropulsionWheel[i].Velocity_Set;
      }

      AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target_Last = AGV_RoundingToNearest_Handle->SteeringWheel[i].Angle_Target;  
  }
}


/**
  * @brief          AGV舵向电机目标角度处理
  * @param[out]     //AGV_Handle_Typedef *AGV_SteerWheel_TargetAngle_Handle
  * @retval         none
  * @note           目标值 = 设置值 + 增量式 + 初始偏置
  * @note           +初始偏置
  * @note           因为目标输出是增量式角度，所以需要做处理，使用的float(-2^23-2^23),无须担心补码的情况
  */
void AGV_SteerWheel_TargetAngle_Handle(AGV_Handle_Typedef *AGV_SteerWheel_TargetAngle_Handle)
{
    for(uint8_t i = 0;i < 4;i ++)
    {
        if(AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target_Last - AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target > 180)
        {
          AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Round_Cnt_Target --;
        }
        else if(AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target_Last - AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target < -180)
        {
          AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Round_Cnt_Target ++;
        }

         AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target_beforehand =  AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Set + AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Round_Cnt_Target*360 + 
                                                                              AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Bias;
    }
}

/**
  * @brief          AGV PID计算
  * @param[out]     AGV_Handle_Typedef *AGV_PID_Cal
  * @retval         none
  */
void AGV_PID_Cal(AGV_Handle_Typedef *AGV_PID_Cal)
{
 
  for(uint8_t i = 0;i < 4;i ++)
  { 
    //限制Target
    AGV_PID_Cal->SteeringWheel[i].Angle_Target = AGV_PID_Cal->SteeringWheel[i].Angle_Target;
    AGV_PID_Cal->PropulsionWheel[i].Velocity_Target = fp32_constrain(AGV_PID_Cal->PropulsionWheel[i].Velocity_Target,AGV_PID_Cal->PropulsionWheel[i].v_max,AGV_PID_Cal->PropulsionWheel[i].v_min);
    //PID校准
    PID_calc(&AGV_PID_Cal->PropulsionWheel[i].pid,AGV_PID_Cal->PropulsionWheel[i].Velocity_Now,AGV_PID_Cal->PropulsionWheel[i].Velocity_Target);
    PID_calc(&AGV_PID_Cal->SteeringWheel[i].OutPid,AGV_PID_Cal->SteeringWheel[i].Angle_Now,AGV_PID_Cal->SteeringWheel[i].Angle_Target);
    PID_calc(&AGV_PID_Cal->SteeringWheel[i].InnerPid,AGV_PID_Cal->SteeringWheel[i].Velocity_Now,AGV_PID_Cal->SteeringWheel[i].OutPid.out);
    //赋值给电流值
    AGV_PID_Cal->SteeringWheel[i].motor_info.give_current = (int16_t)(AGV_PID_Cal->SteeringWheel[i].InnerPid.out);
    AGV_PID_Cal->PropulsionWheel[i].motor_info.give_current = (int16_t)(AGV_PID_Cal->PropulsionWheel[i].pid.out);
    

  }

}







