/*
 * @Author: PLIncuBus wewean@yeah.net
 * @Date: 2025-07-01 10:21:15
 * @LastEditors: PLIncuBus wewean@yeah.net
 * @LastEditTime: 2025-07-06 11:10:22
 * @FilePath: \SUEP_SENTRY_CHASSIS\application\AGV_Chassis_param.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __AGV_CHASSIS_PARAM_H
#define __AGV_CHASSIS_PARAM_H

//初始化PID
//舵向电机内环速度环
#define AGV_SteeringWheel_innner_Kp 0
#define AGV_SteeringWheel_innner_Ki 0
#define AGV_SteeringWheel_innner_Kd 0
#define AGV_SteeringWheel_inner_max_out 0
#define AGV_SteeringWheel_inner_min_out 0
//舵向电机外环位置环
#define AGV_SteeringWheel_out_Kp 2000
#define AGV_SteeringWheel_out_Ki 0
#define AGV_SteeringWheel_out_Kd 0
#define AGV_SteeringWheel_out_max_out 16000
#define AGV_SteeringWheel_out_min_out 0
//行进轮电机速度环
#define AGV_PropulsionWheel_Kp 5000
#define AGV_PropulsionWheel_Ki 0
#define AGV_PropulsionWheel_Kd 0
#define AGV_PropulsionWheel_max_out 16000
#define AGV_PropulsionWheel_min_out 0

//初始化舵向电机偏置
#define AGV_Chassis_SteeringWheel1_Bias 36.8701172
#define AGV_Chassis_SteeringWheel2_Bias 29.0917969
#define AGV_Chassis_SteeringWheel3_Bias -26.5869141
#define AGV_Chassis_SteeringWheel4_Bias -27.2460938

//死区限制
#define AGV_MAX_VX 2.0f
#define AGV_MAX_VY 1.5f

//底盘轮中心半径
#define AGV_Chassis_Radius 0.212
//m3508 rmp change to chassis speed,
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

#define RADIAN_TO_DEGREE 57.29577951






#endif