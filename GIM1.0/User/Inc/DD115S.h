#ifndef __DD115S_H
#define __DD115S_H

#include <stdint.h>
#include "main.h"

//  电机硬件参数定义 
#define MOTOR_COUNT             4           // 4个驱动电机 [B,D,F,H]

//  电机速度限制
#define DD115S_MAX_RPM          200         // 实际使用最大转速（接近空载转速200RPM，安全）
#define DD115S_MIN_RPM          -200        // 实际使用最小转速
#define DD115S_RATED_RPM        115         // 额定转速（最佳工作点）

//  控制死区参数 
#define SBUS_CHANNEL_DEADZONE   41          // SBUS通道死区（避免中位漂移）

//  电机ID定义 
#define MOTOR_ID_B              0x01        // B轮电机ID（前左驱动轮）
#define MOTOR_ID_D              0x02        // D轮电机ID（前右驱动轮）
#define MOTOR_ID_F              0x03        // F轮电机ID（后右驱动轮）
#define MOTOR_ID_H              0x04        // H轮电机ID（后左驱动轮）

//  RS485通信协议定义 
#define RS485_CMD_CONTROL       0x64        // 控制指令
#define RS485_CMD_MODE_SET      0xA0        // 模式设置指令
#define RS485_CMD_QUERY         0x74        // 状态查询指令
#define RS485_TIMEOUT_MS        50          // 通信超时时间(ms)
#define RS485_TX_DELAY_MS       5           // 发送延迟(ms)
#define RS485_MOTOR_DELAY_MS    10          // 电机间发送间隔(ms)

typedef struct MovingWheel_t_Struct MovingWheel_t;



//电机模式
typedef enum {
    MOTOR_MODE_CURRENT      = 0x01,         // 电流环模式
    MOTOR_MODE_SPEED        = 0x02,         // 速度环模式  
    MOTOR_MODE_POSITION     = 0x03          // 位置环模式
} MotorMode_t;

//  电机故障码定义 
#define MOTOR_FAULT_NONE        0x00        // 无故障
#define MOTOR_FAULT_SENSOR      0x01        // BIT0: 传感器故障
#define MOTOR_FAULT_OVERCURRENT 0x02        // BIT1: 过流故障
#define MOTOR_FAULT_PHASE_OC    0x04        // BIT2: 相电流过流
#define MOTOR_FAULT_STALL       0x08        // BIT3: 堵转故障
#define MOTOR_FAULT_OVERTEMP    0x10        // BIT4: 过温故障

//  电机状态结构体 
typedef struct {
    MotorMode_t mode;                       // 当前工作模式
    int16_t speed_fb;                       // 速度反馈 (rpm)
    int16_t current_fb;                     // 转矩电流反馈 (原始值)
    uint16_t position_fb;                   // 位置反馈 (0-32767对应0-360°)
    uint8_t temperature;                    // 绕组温度 (℃)
    uint8_t fault_code;                     // 故障码
    uint32_t last_update_ms;                // 最后更新时间戳 (用于超时检测)
    uint8_t comm_error_count;               // 通信错误计数
} MotorStatus_t;

//  全局变量声明 
extern int16_t rpm_motors[MOTOR_COUNT];                 // 4个电机转速数组 [B,D,F,H]
extern MotorStatus_t g_motor_status[MOTOR_COUNT];      // 4个电机状态数组

//  CRC计算函数 
uint8_t crc8_maxim(uint8_t *data, uint8_t len);

//  电机参数处理函数 
int16_t MovingMotor_LimitRPM(int16_t rpm);              // 限制转速在±200范围内（与.c文件函数名一致）

//  单电机控制函数 
void Motor_SendSpeed(uint8_t motor_id, int16_t rpm);                // 发送速度指令
void Motor_SendCurrent(uint8_t motor_id, int16_t current);          // 发送电流指令
void Motor_SendPosition(uint8_t motor_id, uint16_t position);       // 发送位置指令
void Motor_SetMode(uint8_t motor_id, MotorMode_t mode);             // 设置电机模式
void Motor_QueryStatus(uint8_t motor_id);                           // 查询电机状态
void Motor_Brake(uint8_t motor_id);                                 // 刹车单个电机

//  多电机控制函数 
void Motor_SendSpeedAll(int16_t rpm_B, int16_t rpm_D, int16_t rpm_F, int16_t rpm_H);
void Motor_SetModeAll(MotorMode_t mode);
void Motor_QueryStatusAll(void);
void Motor_BrakeAll(void);                                          // 刹车所有电机

//  数组方式控制函数 
void Motor_SendSpeedArray(int16_t rpm_array[MOTOR_COUNT]);

//  反馈处理函数 
void Motor_ParseControlFeedback(uint8_t *rx_data, uint8_t motor_index);
void Motor_ParseQueryFeedback(uint8_t *rx_data, uint8_t motor_index);
void Motor_UpdateFeedback(void);                        // 更新反馈到 g_moving_wheel

//  故障诊断函数 
const char* Motor_GetFaultString(uint8_t fault_code);
uint8_t Motor_HasFault(uint8_t motor_index);
void Motor_ClearCommErrorCount(uint8_t motor_index);

//  系统级控制函数 
void Motor_SystemInit(void);                            // 电机系统初始化
void Motor_EmergencyStop(void);                         // 紧急停止所有电机
void Motor_UpdateControl(void);                         // 根据g_moving_wheel更新电机控制

void MotorDriver_UpdateMotors(MovingWheel_t *moving_wheel);
void MotorDriver_StopAll(void);
void MotorDriver_SetXLock(void);


#endif // __DD115S_H






