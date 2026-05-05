#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "main.h"

//  遥控器连接和超时检测参数 
#define RC_INIT_STABILIZATION_MS    1000    // 初始化稳定期时间(ms)
#define RC_CONNECTION_TIMEOUT_MS    500     // 连接超时时间(ms)
#define RC_DATA_CHANGE_THRESHOLD    10      // 数据变化检测阈值

// 遥控器数据结构体 
typedef struct RemoteControl_t_Struct
{
    // 连接状态 
    uint8_t IsRCConnected;              // 遥控器连接状态：1=连接，0=失联
    uint8_t InitializationComplete;    // 初始化完成标志：1=完成，0=未完成
    uint32_t LastUpdateTime;           // 最后更新时间戳（毫秒）
    
    // 原始SBUS数据 
    uint16_t CH0_Steering;             // CH0: 转向原始值 (200-1800)
    uint16_t CH1_Throttle;             // CH1: 前进后退原始值 (200-1800)
    uint16_t CH4_MoveMode;             // CCH4: 运动模式原始值
    uint16_t CH8_Reset;                // BCH8: 刷新复位原始值
    uint16_t CH9_SystemMode;           // ACH9: 系统模式原始值
    
    // 归一化摇杆数据 
    float LeftY;                       // 左摇杆Y轴：前进后退控制 (-1.0 ~ +1.0)
    float RightX;                      // 右摇杆X轴：转向控制 (-1.0 ~ +1.0)

    // 解析后的模式状态 
    uint8_t SystemMode;                // 系统模式：0=锁定，1=运动
    uint8_t MoveMode;                  // 运动模式：0=阿克曼，1=蟹行，2=旋转
    uint8_t ResetMode;                 // 复位模式：0=关闭，1=触发复位
    
    // 模式变化事件标志 
    uint8_t SystemModeChanged;         // 系统模式变化标志：1=变化，0=无变化
    uint8_t MoveModeChanged;           // 运动模式变化标志：1=变化，0=无变化
    uint8_t ResetModeChanged;          // 复位触发标志：1=触发，0=无动作
    
    // 数据变化检测 
    uint8_t RCStatic;                  // 遥控器静态标志：1=静止，0=有变化
    
} RemoteControl;

extern RemoteControl RC;


void RCInit(void);                          // 遥控器安全初始化
void RCGetValue(void);                      // 获取和处理遥控器数据
uint8_t RCIsChanged(void);                  // 检测遥控器数据是否变化
void RCCheckConnection(void);               // 检查连接状态和超时处理
uint8_t RCIsInitialized(void);              // 检查初始化是否完成

//复位函数
uint8_t RC_IsResetTriggered(void);          // 获取当前复位触发状态
void RC_ClearResetTrigger(void);            // 手动清除复位触发标志

#endif // __REMOTE_CONTROL_H



