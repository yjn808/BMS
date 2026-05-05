#include "CANDrive.h"

#ifdef HAL_FDCAN_MODULE_ENABLED  // 仅在HAL库启用FDCAN模块时编译

uint8_t FDCAN2_buff[8];         //FDCAN2接收缓冲区

void CanFilter_Init(void) 
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;// 配置标准帧滤波器（11位ID）
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x000007FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;//配置扩展帧滤波器（29位ID）
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}


// 发送CAN标准数据帧
//(FDCAN句柄指针（&hfdcan2）CAN标准帧ID（11位有效，范围0x000-0x7FF）指向8字节数据数组的指针)
HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t Identifier, uint8_t *msg) {
    // 配置发送报文头
    FDCAN_TxHeaderTypeDef CAN_Tx = {
        .Identifier = Identifier,                   // CAN ID（标准帧11位）
        .IdType = FDCAN_STANDARD_ID,                // 帧类型：标准帧（Standard Frame）
        .TxFrameType = FDCAN_DATA_FRAME,            // 帧性质：数据帧（非远程帧）
        .DataLength = FDCAN_DLC_BYTES_8,            // 数据长度代码：8字节（DLC=8）
        .ErrorStateIndicator = FDCAN_ESI_PASSIVE,   // 错误状态指示：被动错误（Error Passive）
        .FDFormat = FDCAN_CLASSIC_CAN,              // CAN格式：经典CAN（非CAN FD）
        .BitRateSwitch = FDCAN_BRS_OFF,             // 位速率切换：关闭（CAN FD才支持数据段加速）
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,   // 发送事件：不记录到Tx Event FIFO
        . MessageMarker = 0                          // 消息标记：用户自定义标识（可用于追踪/调试）
    };
    
    // 计算延迟补偿值 = 数据段预分频器 × 数据段时间段1
    // 用于补偿CAN收发器的信号传输延迟，提高高速通信可靠性
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 
        hfdcan2.Init. DataPrescaler * hfdcan2.Init.DataTimeSeg1,  // 延迟补偿值（单位：时钟周期）
        0);  // 偏移量（通常为0）
    
    // 使能发送延迟补偿功能
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);

    /* ========== 添加消息到发送FIFO/队列 ========== */
    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_Tx, msg);
    return err;  // 返回发送状态
}



uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf) {
    FDCAN_RxHeaderTypeDef CAN_Rx = { 0 };
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_Rx, buf) == HAL_OK) {
        return CAN_Rx.Identifier;        
    }
    else {
        Error_Handler();
        return 0;
    }
}

#endif
