/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
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
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    AGV_Chassis_Init(&AGV_Handle);
    // //make sure all chassis motor is online,
    // //�жϵ��̵���Ƿ�����
    // while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    // {
    //     vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    // }

    while (1)
    {  
//      //channel value and keyboard value change to speed set-point, in general
//      //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
//      AGV_chassis_rc_to_control_vector(&AGV_Handle.AGV_Mov.vx_set, &AGV_Handle.AGV_Mov.vy_set, &AGV_Handle);
      //AGV��������
      AGV_Feedback_Update(&AGV_Handle);
//      //�˶�ѧ�����
//      AGV_InverseKinematics(&AGV_Handle);
//      //AGV������������ת�Ƕȴ���
//      AGV_SteerWheel_EcdToAngle_Handle(&AGV_Handle);
//      //AGV������Ŀ��Ƕȴ���
//      AGV_SteerWheel_TargetAngle_Handle(&AGV_Handle);
//      //�ͽ�ԭ����
//      AGV_RoundingToNearest_Handle(&AGV_Handle);
//      //PID����
//      AGV_PID_Cal(&AGV_Handle);
//      CAN_cmd_steer(AGV_Handle.SteeringWheel[0].motor_info.give_current, AGV_Handle.SteeringWheel[1].motor_info.give_current,
//                    AGV_Handle.SteeringWheel[2].motor_info.give_current, AGV_Handle.SteeringWheel[3].motor_info.give_current); 
//      CAN_cmd_chassis(AGV_Handle.PropulsionWheel[0].motor_info.give_current,AGV_Handle.PropulsionWheel[1].motor_info.give_current,
//                      AGV_Handle.PropulsionWheel[2].motor_info.give_current,AGV_Handle.PropulsionWheel[3].motor_info.give_current);          

        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


