#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    
    CAN_FilterTypeDef can_filter_st1;
    //手册中的CAN2_SB = 28 时，可以使用CAN1的所有过滤器
    //这里配置为can1:0-14 can2:14-28
    can_filter_st1.SlaveStartFilterBank = 14;
    can_filter_st1.FilterActivation = ENABLE;
    can_filter_st1.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st1.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st1.FilterIdHigh = 0x0000;
    can_filter_st1.FilterIdLow = 0x0000;
    can_filter_st1.FilterMaskIdHigh = 0x0000;
    can_filter_st1.FilterMaskIdLow = 0x0000;
    can_filter_st1.FilterBank = 0;
    //CAN1使用FIFO0
    can_filter_st1.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);



    CAN_FilterTypeDef can_filter_st2;
    //手册中的CAN2_SB = 28 时，可以使用CAN1的所有过滤器
    //这里配置为can1:0-14 can2:14-28
    can_filter_st2.SlaveStartFilterBank = 14;
    can_filter_st2.FilterActivation = ENABLE;
    can_filter_st2.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st2.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st2.FilterIdHigh = 0x0000;
    can_filter_st2.FilterIdLow = 0x0000;
    can_filter_st2.FilterMaskIdHigh = 0x0000;
    can_filter_st2.FilterMaskIdLow = 0x0000;
    can_filter_st2.FilterBank = 14;
    //CAN1使用FIFO1
    can_filter_st2.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}
