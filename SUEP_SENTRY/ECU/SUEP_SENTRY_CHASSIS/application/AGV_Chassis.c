#include "AGV_Chassis.h"
#include "AGV_Chassis_param.h"
#include "arm_math.h"
#include "chassis_task.h"
#include "user_lib.h"
#include "CAN_receive.h"




AGV_Handle_Typedef AGV_Handle;



#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }





/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void AGV_chassis_rc_to_control_vector(AGV_Handle_Typedef *chassis_move_rc_to_vector)
{
//    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
//    {
//        return;
//    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    // //keyboard set speed set-point
    // //键盘控制
    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->AGV_Mov.vx_max;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->AGV_Mov.vx_min;
    // }

    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    // {
    //     //vy_set_channel = chassis_move_rc_to_vector->vy_max;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    // {
    //     //vy_set_channel = chassis_move_rc_to_vector->vy_min;
    // }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vy.out = 0.0f;
    }

    chassis_move_rc_to_vector->AGV_Mov.vx_set = chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vx.out;
    chassis_move_rc_to_vector->AGV_Mov.vy_set = chassis_move_rc_to_vector->AGV_Mov.chassis_cmd_slow_set_vy.out;
    chassis_move_rc_to_vector->AGV_Mov.vz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
  }


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
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[0][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[0][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[0][1],&AGV_InverseKinematics->PropulsionWheel[0].Velocity_Set);
    AGV_InverseKinematics->PropulsionWheel[0].Velocity_Set = AGV_InverseKinematics->PropulsionWheel[0].Velocity_Set*AGV_InverseKinematics->PropulsionWheel[0].direction;

    AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] =  AGV_InverseKinematics->AGV_Mov.vy_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[1][1] =  AGV_InverseKinematics->AGV_Mov.vx_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[1][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[1][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[1][1],&AGV_InverseKinematics->PropulsionWheel[1].Velocity_Set);
    AGV_InverseKinematics->PropulsionWheel[1].Velocity_Set = AGV_InverseKinematics->PropulsionWheel[1].Velocity_Set*AGV_InverseKinematics->PropulsionWheel[1].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] =  AGV_InverseKinematics->AGV_Mov.vy_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[2][1] =  AGV_InverseKinematics->AGV_Mov.vx_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[2][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[2][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[2][1],&AGV_InverseKinematics->PropulsionWheel[2].Velocity_Set);
    AGV_InverseKinematics->PropulsionWheel[2].Velocity_Set = AGV_InverseKinematics->PropulsionWheel[2].Velocity_Set*AGV_InverseKinematics->PropulsionWheel[2].direction;
   
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] =  AGV_InverseKinematics->AGV_Mov.vy_set + AGV_InverseKinematics->AGV_Mov.vz_set*arm_cos_f32(0.78539825f)*AGV_Chassis_Radius;
    AGV_InverseKinematics->AGV_Mov.Sqrt[3][1] =  AGV_InverseKinematics->AGV_Mov.vx_set - AGV_InverseKinematics->AGV_Mov.vz_set*arm_sin_f32(0.78539825f)*AGV_Chassis_Radius;                                                         
    arm_sqrt_f32(AGV_InverseKinematics->AGV_Mov.Sqrt[3][0]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][0] + AGV_InverseKinematics->AGV_Mov.Sqrt[3][1]*AGV_InverseKinematics->AGV_Mov.Sqrt[3][1],&AGV_InverseKinematics->PropulsionWheel[3].Velocity_Set); 
    AGV_InverseKinematics->PropulsionWheel[3].Velocity_Set = AGV_InverseKinematics->PropulsionWheel[3].Velocity_Set*AGV_InverseKinematics->PropulsionWheel[3].direction;
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
      AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now =(float)(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].motor_info.chassis_motor_measure->ecd) / 8192 * 360;
      if(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now - AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Last > 180)
      {
        AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now --;
      }
      else if(AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now - AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Last < -180)
      {
        AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now ++;
      }
      AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Angle_Now = AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Round_Cnt_Now*360 + AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Ecd_Now + AGV_SteerWheel_EcdToAngle_Handle->SteeringWheel[i].Angle_Bias;
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
        if(AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target_Last - AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Set > 180)
        {
          AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Round_Cnt_Target --;
        }
        else if(AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Target_Last - AGV_SteerWheel_TargetAngle_Handle->SteeringWheel[i].Angle_Set < -180)
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
    // AGV_PID_Cal->PropulsionWheel[i].Velocity_Target = fp32_constrain(AGV_PID_Cal->PropulsionWheel[i].Velocity_Target,AGV_PID_Cal->PropulsionWheel[i].v_max,AGV_PID_Cal->PropulsionWheel[i].v_min);
    //PID校准
    PID_calc(&AGV_PID_Cal->PropulsionWheel[i].pid,AGV_PID_Cal->PropulsionWheel[i].Velocity_Now,AGV_PID_Cal->PropulsionWheel[i].Velocity_Target);
    PID_calc(&AGV_PID_Cal->SteeringWheel[i].OutPid,AGV_PID_Cal->SteeringWheel[i].Angle_Now,AGV_PID_Cal->SteeringWheel[i].Angle_Target);
    PID_calc(&AGV_PID_Cal->SteeringWheel[i].InnerPid,AGV_PID_Cal->SteeringWheel[i].Velocity_Now,AGV_PID_Cal->SteeringWheel[i].OutPid.out);
    //赋值给电流值
    AGV_PID_Cal->SteeringWheel[i].motor_info.give_current = (int16_t)(AGV_PID_Cal->SteeringWheel[i].InnerPid.out);
    AGV_PID_Cal->PropulsionWheel[i].motor_info.give_current = (int16_t)(AGV_PID_Cal->PropulsionWheel[i].pid.out);
    

  }

}







