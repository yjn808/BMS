#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"


// 串口配置：100000波特率，8位数据位（STM32需设置9位），偶校验，2位停止位
#define SBUS_FRAME_SIZE 25              // SBUS帧长度：25字节
#define SBUS_HEADER 0x0F                // 帧头标识
#define SBUS_FOOTER 0x00                // 帧尾标识
#define SBUS_CHANNEL_COUNT 18           // 通道总数（SBUS协议最多支持18通道）

//遥控器通道映射
#define RC_CH_STEERING              0       // CH0: 转向
#define RC_CH_THROTTLE              1       // CH1: 前进后退
#define RC_CH_MOVE_MODE_SWITCH      4       // CCH4: 运动模式
#define RC_CH_RESET_SWITCH          8       // BCH8: 刷新复位
#define RC_CH_SYSTEM_SWITCH         9       // ACH9: 解锁锁定

//CH0: 左右转向参数（CAN电机机控制）
#define CH0_STEERING_MIN            200     // 最小值（右转）
#define CH0_STEERING_CENTER         1000    // 中位值
#define CH0_STEERING_MAX            1800    // 最大值（左转）
#define CH0_STEERING_DEADZONE       50      // 死区范围


//CH1: 加速刹车参数
#define CH1_THROTTLE_MIN            200     // 最小值（后退）
#define CH1_THROTTLE_CENTER         950     // 中位值（停止）
#define CH1_THROTTLE_MAX            1760    // 最大值（前进）
#define CH1_THROTTLE_DEADZONE       50      // 死区范围


//  ACH9系统模式
#define ACH9_SYSTEM_LOCKED          200     // 锁定模式
#define ACH9_SYSTEM_MOVING          1800    // 运动模式
#define ACH9_SYSTEM_TOLERANCE       300     // 挡位检测容差


//  BCH8刷新复位
#define BCH8_RESET1                 200     
#define BCH8_RESET2                 1800    
#define BCH8_RESET_TOLERANCE        300     


//  CCH4运动模式
#define CCH4_MOVE_ACKERMANN         200     // 双阿克曼
#define CCH4_MOVE_CRABBING          1000    // 蟹行模式
#define CCH4_MOVE_ROTATION          1800    // 旋转模式
#define CCH4_MOVE_TOLERANCE         300     // 挡位检测容差


extern uint16_t ch[18];                 // 存储18个通道的解析值（实际只用16个）

void Sbus_Parse(uint8_t *buf, uint16_t *ch);//解析SBUS数据帧


uint8_t Sbus_GetSystemMode(uint16_t ach9_value);          // ACH9: 系统模式
uint8_t Sbus_GetMoveMode(uint16_t cch4_value);            // CCH4: 运动模式
uint8_t Sbus_GetResetMode(uint16_t bch8_value);           // BCH8: 复位

uint8_t Sbus_IsFrameValid(uint8_t *buf);// 检查SBUS数据帧是否有效

float Sbus_MapThrottle(uint16_t ch1_value);//将CH1油门摇杆值映射到-1.0~+1.0范围（带死区处理）
float Sbus_MapSteering(uint16_t ch0_value);//将CH0转向摇杆值映射到-1.0~+1.0范围（带死区处理）


#endif // __SBUS_H




