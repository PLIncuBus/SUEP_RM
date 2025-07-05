/*
 * @Author: PLIncuBus wewean@yeah.net
 * @Date: 2025-07-04 23:26:45
 * @LastEditors: PLIncuBus wewean@yeah.net
 * @LastEditTime: 2025-07-05 17:01:49
 * @FilePath: \ECU\SUEP_SENTRY_CHASSIS\application\UpperMonitor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "UpperMonitor.h"
#include "AGV_Chassis.h"
#include "string.h"

UpperMonitor_Handle_Typedef UpperMonitor_Handle;

/**
 * @brief           上位机发送拼接函数
 * @param           //Send_Theme发送主题
 */
#define UpperMonitor_Com_Send(Send_Theme)                                             \
        UpperMonitor_##Send__Theme##_Send_Hook(&UpperMonitor_Handle);                 \
        UpperMonitor_Cmd_Sned(&UpperMonitor_Handle);


/**
 * @brief           上位机数据发送
 * @param           //UpperMonitor_Cmd_Send 
 */
static void UpperMonitor_Cmd_Send(UpperMonitor_Handle_Typedef *UpperMonitor_Cmd_Send)
{
    #if (UpperMonitor_Def == VOFA_JustFlow)
        const uint8_t UpperMonitor_VOFA_JustFlow_Tail_Buff[4] = { 0x00 , 0x00 , 0x80 , 0x7F };     
        memcpy( UpperMonitor_Cmd_Send->Usart_Send_Buff + UpperMonitor_Cmd_Send->Usart_Send_Index, UpperMonitor_VOFA_JustFlow_Tail_Buff , 4 * sizeof(uint8_t));
        UpperMonitor_Cmd_Send->Usart_Send_Index += 4*sizeof(uint8_t); 
        
        UpperMonitor_Cmd_Send->Usart_Send_Index = 0;
    #endif
}


/**
 * @brief           模拟阶跃信号
 * @param[in]       //float *Step_Signal_array,uint8_t Step_Siganl_size,uint8_t repeat_cnt
 * @return          float 
 */
static float UpperMonitor_Step_Signal_Sim(float *Step_Signal_array,uint8_t Step_Siganl_size,uint8_t repeat_cnt)
{
    static uint8_t index,scale;
    index = scale / repeat_cnt;
    scale ++;
    if(scale > Step_Siganl_size) index = 0;
    return Step_Signal_array[index];   
}


/**
 * @brief           上位机PID发送钩子函数
 * @param[in]       
 * @note            
 */

static void UpperMonitor_Motor_PID_Send_Hook(UpperMonitor_Handle_Typedef *UpperMonitor_Handle_Motor_PID_Hook)
{
    //阶跃信号数值
    const float Step_Signal[] = {3,2,1,3,5,1,0};
    //阶跃信号重复次数
    const uint8_t Step_Signal_repeat_cnt = 100;
    //上位机包
    static float Usart_Send_Buff[10];

    #if(Chassis_Def == AGV_Handle_Typedef)
        //单环速度环
        #if (UpperMonitor_Motor_PID_Loop_Def == Single_Velocity_Loop) 
        Usart_Send_Buff[0] = UpperMonitor_Step_Signal_Sim((float *)Step_Signal,sizeof(Step_Signal)/sizeof(float),Step_Signal_repeat_cnt);
        Usart_Send_Buff[1] = AGV_Handle.PropulsionWheel[0].Velocity_Now;   
        
        //双环内环速度环
        #elif (UpperMonitor_Motor_PID_Loop_Def == Double_Inner_Loop)
        Usart_Send_Buff[0] = UpperMonitor_Step_Signal_Sim(Step_Signal,sizeof(Step_Signal)/sizeof(float),Step_Signal_repeat_cnt);
        Usart_Send_Buff[1] = AGV_Handle.SteeringWheel[0].Velocity_Now;

        //双环外环角度环
        #elif (UpperMonitor_Motor_PID_Loop_Def == Double_Out_Loop)
        Usart_Send_Buff[0] = UpperMonitor_Step_Signal_Sim(Step_Signal,sizeof(Step_Signal)/sizeof(float),Step_Signal_repeat_cnt);
        Usart_Send_Buff[1] = AGV_Handle.SteeringWheel[0].Angle_Now;

        #endif
         
    memcpy(UpperMonitor_Handle_Motor_PID_Hook->Usart_Send_Buff,Usart_Send_Buff,2*sizeof(float));
    UpperMonitor_Handle_Motor_PID_Hook->Usart_Send_Index += 2*sizeof(float);

    #endif   
}

/**
 * @brief           上位机PID接收钩子函数
 * @param           
 * 
 */
static void UpperMonitor_Motor_PID_Receive_Hook(UpperMonitor_Handle_Typedef *UpperMonitor_Motor_PID_Receive_Hook)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacaket = 0;
    #if (UpperMonitor_Def == VOFA_JustFlow)
        // if(USART)

    #endif

}



