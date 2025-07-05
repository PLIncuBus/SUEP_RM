#ifndef __UPPERMONITOR_H
#define __UPPERMONITOR_H

#include "bsp_usart.h"
#include "main.h"

#define UpperMonitor_Def                    VOFA
#define Chassis_Def                         AGV_Handle_Typedef
#define UpperMonitor_Motor_PID_Loop_Def     Single_Velocity_Loop

enum{
    VOFA_JustFlow,
}UpperMonitor_enum;


enum{
    Single_Velocity_Loop,
    Double_Inner_Loop,
    Double_Out_Loop,
}UpperMonitor_Motor_PID_Loop_enum;


typedef struct 
{
    uint8_t Usart_Send_Buff[100];
    uint8_t Usart_Receive_Buff[100];
    uint8_t Usart_Send_Index;

    UART_HandleTypeDef *huart;
 
}UpperMonitor_Handle_Typedef;


#endif