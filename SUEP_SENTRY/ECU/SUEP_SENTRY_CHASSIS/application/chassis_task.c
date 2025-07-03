/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "AGV_Chassis.h"

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



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif





/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    AGV_Chassis_Init(&AGV_Handle);
    // //make sure all chassis motor is online,
    // //判断底盘电机是否都在线
    // while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    // {
    //     vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    // }

    while (1)
    {  
//      //channel value and keyboard value change to speed set-point, in general
//      //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
//      AGV_chassis_rc_to_control_vector(&AGV_Handle.AGV_Mov.vx_set, &AGV_Handle.AGV_Mov.vy_set, &AGV_Handle);
      //AGV参数反馈
      AGV_Feedback_Update(&AGV_Handle);
//      //运动学逆结算
//      AGV_InverseKinematics(&AGV_Handle);
//      //AGV舵向电机编码器转角度处理
//      AGV_SteerWheel_EcdToAngle_Handle(&AGV_Handle);
//      //AGV舵向电机目标角度处理
//      AGV_SteerWheel_TargetAngle_Handle(&AGV_Handle);
//      //就近原则处理
//      AGV_RoundingToNearest_Handle(&AGV_Handle);
//      //PID计算
//      AGV_PID_Cal(&AGV_Handle);
//      CAN_cmd_steer(AGV_Handle.SteeringWheel[0].motor_info.give_current, AGV_Handle.SteeringWheel[1].motor_info.give_current,
//                    AGV_Handle.SteeringWheel[2].motor_info.give_current, AGV_Handle.SteeringWheel[3].motor_info.give_current); 
//      CAN_cmd_chassis(AGV_Handle.PropulsionWheel[0].motor_info.give_current,AGV_Handle.PropulsionWheel[1].motor_info.give_current,
//                      AGV_Handle.PropulsionWheel[2].motor_info.give_current,AGV_Handle.PropulsionWheel[3].motor_info.give_current);          

        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


