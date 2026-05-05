#include "DD115S.h"
 

// ========== 全局变量定义 ==========
int16_t rpm_motors[MOTOR_COUNT] = {0, 0, 0, 0};        // 4个电机转速数组 [B,D,F,H]
MotorStatus_t g_motor_status[MOTOR_COUNT];       // 4个电机状态数组

// ========== 外部变量引用 ==========
extern MovingWheel_t g_moving_wheel;
extern UART_HandleTypeDef huart2;


void RS485_Send(uint8_t *data, uint8_t len)
{  
    HAL_UART_Transmit(&huart2, data, len, 100);// 硬件自动控制DE引脚，直接发送即可
    HAL_Delay(5); // 短暂延时确保发送完成
}


// RS485接收函数（带超时）
 HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    return HAL_UART_Receive(&huart2, data, len, timeout_ms);
}






// CRC8-MAXIM校验算法（用于DDSM115通信协议）
// 多项式: x^8 + x^5 + x^4 + 1 (0x8C)
uint8_t crc8_maxim(uint8_t *data, uint8_t len)  
{
    uint8_t crc = 0x00;
    while (len--) 
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) 
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}


// 限制RPM值在 [-200, +200] 范围内（安全范围）
int16_t MovingMotor_LimitRPM(int16_t rpm)
{
    if (rpm > DD115S_MAX_RPM) return DD115S_MAX_RPM;
    if (rpm < DD115S_MIN_RPM) return DD115S_MIN_RPM;
    return rpm;
}



// 单电机发送速度控制指令
void Motor_SendSpeed(uint8_t motor_id, int16_t rpm)
{
    rpm = MovingMotor_LimitRPM(rpm);  // 限幅

    uint8_t tx[10];
    tx[0] = motor_id;                       // 电机ID
    tx[1] = RS485_CMD_CONTROL;              // 控制指令 (0x64)
    tx[2] = (rpm >> 8) & 0xFF;              // 速度高8位
    tx[3] = rpm & 0xFF;                     // 速度低8位
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;                           // 加速时间（0=默认）
    tx[7] = 0x00;                           // 刹车标志（0=正常）
    tx[8] = 0x00;
    tx[9] = crc8_maxim(tx, 9);              // CRC校验

    RS485_Send(tx, 10);
}



//单电机发送电流控制指令
void Motor_SendCurrent(uint8_t motor_id, int16_t current)
{
    uint8_t tx[10];
    tx[0] = motor_id;
    tx[1] = RS485_CMD_CONTROL;              // 控制指令 (0x64)
    tx[2] = (current >> 8) & 0xFF;
    tx[3] = current & 0xFF;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    tx[8] = 0x00;
    tx[9] = crc8_maxim(tx, 9);

    RS485_Send(tx, 10);
}

// 单电机发送位置控制指令
void Motor_SendPosition(uint8_t motor_id, uint16_t position)
{
    uint8_t tx[10];
    tx[0] = motor_id;
    tx[1] = RS485_CMD_CONTROL;              // 控制指令 (0x64)
    tx[2] = (position >> 8) & 0xFF;
    tx[3] = position & 0xFF;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    tx[8] = 0x00;
    tx[9] = crc8_maxim(tx, 9);

    RS485_Send(tx, 10);
}

// 单电机设置电机工作模式
void Motor_SetMode(uint8_t motor_id, MotorMode_t mode)
{
    uint8_t tx[10];
    tx[0] = motor_id;
    tx[1] = RS485_CMD_MODE_SET;             // 模式切换指令 (0xA0)
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    tx[8] = 0x00;
    tx[9] = mode;                           // 模式值 (0x01/0x02/0x03)

    RS485_Send(tx, 10);
    HAL_Delay(100);                         // 模式切换需要时间
    
    // 更新状态记录
    if (motor_id >= 1 && motor_id <= MOTOR_COUNT) {
        g_motor_status[motor_id - 1].mode = mode;
    }
}

// 查询电机状态
void Motor_QueryStatus(uint8_t motor_id)
{
    uint8_t tx[10];
    tx[0] = motor_id;
    tx[1] = RS485_CMD_QUERY;                // 查询指令 (0x74)
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0x00;
    tx[8] = 0x00;
    tx[9] = crc8_maxim(tx, 9);

    RS485_Send(tx, 10);

    // 接收反馈
    uint8_t rx[10];
    if (RS485_Receive(rx, 10, RS485_TIMEOUT_MS) == HAL_OK) {
        Motor_ParseQueryFeedback(rx, motor_id - 1);
    } else {
        // 通信失败，增加错误计数
        if (motor_id >= 1 && motor_id <= MOTOR_COUNT) {
            g_motor_status[motor_id - 1].comm_error_count++;
        }
    }
}

// 刹车单个电机（仅速度环模式有效）
void Motor_Brake(uint8_t motor_id)
{
    uint8_t tx[10];
    tx[0] = motor_id;
    tx[1] = RS485_CMD_CONTROL;              // 控制指令 (0x64)
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0x00;
    tx[5] = 0x00;
    tx[6] = 0x00;
    tx[7] = 0xFF;                           // 刹车标志 (0xFF=刹车)
    tx[8] = 0x00;
    tx[9] = crc8_maxim(tx, 9);

    RS485_Send(tx, 10);
}


// 控制4个电机速度
void Motor_SendSpeedAll(int16_t rpm_B, int16_t rpm_D, int16_t rpm_F, int16_t rpm_H)
{
    int16_t rpm_array[MOTOR_COUNT] = {rpm_B, rpm_D, rpm_F, rpm_H};
    Motor_SendSpeedArray(rpm_array);
}

// // 控制4个电机电流
// void Motor_SendCurrentAll(int16_t current_B, int16_t current_D, int16_t current_F, int16_t current_H)
// {
//     int16_t current_array[MOTOR_COUNT] = {current_B, current_D, current_F, current_H};
//     Motor_SendCurrentArray(current_array);
// }

// // 控制4个电机位置
// void Motor_SendPositionAll(uint16_t pos_B, uint16_t pos_D, uint16_t pos_F, uint16_t pos_H)
// {
//     uint16_t position_array[MOTOR_COUNT] = {pos_B, pos_D, pos_F, pos_H};
//     Motor_SendPositionArray(position_array);
// }

// 设置所4个电机相同的工作模式
void Motor_SetModeAll(MotorMode_t mode)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Motor_SetMode(i + 1, mode);
        HAL_Delay(20);
    }
}

// 查询4个电机状态
void Motor_QueryStatusAll(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Motor_QueryStatus(i + 1);
        HAL_Delay(15);
    }
}

// 刹车4个电机（仅速度环模式有效）
void Motor_BrakeAll(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Motor_Brake(i + 1);
        HAL_Delay(RS485_MOTOR_DELAY_MS);
    }
}



// 数组方式发送速度指令 [B,D,F,H]
void Motor_SendSpeedArray(int16_t rpm_array[MOTOR_COUNT])
{
    // 更新全局结构体（传递给 g_moving_wheel）
    g_moving_wheel.B_Vel = (float)rpm_array[0];
    g_moving_wheel.D_Vel = (float)rpm_array[1];
    g_moving_wheel.F_Vel = (float)rpm_array[2];
    g_moving_wheel.H_Vel = (float)rpm_array[3];
    
    // 更新全局数组
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        rpm_motors[i] = rpm_array[i];
    }
    
    // 依次发送给4个电机
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        int16_t rpm = MovingMotor_LimitRPM(rpm_array[i]);
        Motor_SendSpeed(i + 1, rpm);
        HAL_Delay(RS485_MOTOR_DELAY_MS);
    }
}

// // 数组方式发送电流指令
// void Motor_SendCurrentArray(int16_t current_array[MOTOR_COUNT])
// {
//     for (uint8_t i = 0; i < MOTOR_COUNT; i++)
//     {
//         Motor_SendCurrent(i + 1, current_array[i]);
//         HAL_Delay(RS485_MOTOR_DELAY_MS);
//     }
// }

// // 数组方式发送位置指令
// void Motor_SendPositionArray(uint16_t position_array[MOTOR_COUNT])
// {
//     for (uint8_t i = 0; i < MOTOR_COUNT; i++)
//     {
//         Motor_SendPosition(i + 1, position_array[i]);
//         HAL_Delay(RS485_MOTOR_DELAY_MS);
//     }
// }

// ========================================================================
// 反馈数据解析函数
// ========================================================================

// 解析控制反馈数据（协议1：0x64指令返回）
void Motor_ParseControlFeedback(uint8_t *rx_data, uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return;
    
    // 验证CRC
    uint8_t crc_calc = crc8_maxim(rx_data, 9);
    if (crc_calc != rx_data[9]) {
        g_motor_status[motor_index].comm_error_count++;
        return;
    }
    
    // 解析数据
    g_motor_status[motor_index].mode = (MotorMode_t)rx_data[1];
    g_motor_status[motor_index].current_fb = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    g_motor_status[motor_index].speed_fb = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    g_motor_status[motor_index].position_fb = (uint16_t)((rx_data[6] << 8) | rx_data[7]);
    g_motor_status[motor_index].fault_code = rx_data[8];
    g_motor_status[motor_index].last_update_ms = HAL_GetTick();
}

// 解析查询反馈数据（协议2：0x74指令返回）
void Motor_ParseQueryFeedback(uint8_t *rx_data, uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return;
    
    // 验证CRC
    uint8_t crc_calc = crc8_maxim(rx_data, 9);
    if (crc_calc != rx_data[9]) {
        g_motor_status[motor_index].comm_error_count++;
        return;
    }
    
    // 解析数据
    g_motor_status[motor_index].mode = (MotorMode_t)rx_data[1];
    g_motor_status[motor_index].current_fb = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    g_motor_status[motor_index].speed_fb = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    g_motor_status[motor_index].temperature = rx_data[6];  // 温度信息（协议2专有）
    // rx_data[7] 是位置信息（0-255对应0-360°）
    g_motor_status[motor_index].fault_code = rx_data[8];
    g_motor_status[motor_index].last_update_ms = HAL_GetTick();
}

// 更新反馈数据到全局运动控制结构体
void Motor_UpdateFeedback(void)
{
    g_moving_wheel.B_Vel_FB = (float)g_motor_status[0].speed_fb;
    g_moving_wheel.D_Vel_FB = (float)g_motor_status[1].speed_fb;
    g_moving_wheel.F_Vel_FB = (float)g_motor_status[2].speed_fb;
    g_moving_wheel.H_Vel_FB = (float)g_motor_status[3].speed_fb;
}

// ========================================================================
// 故障诊断函数
// ========================================================================

// 获取故障码字符串描述
const char* Motor_GetFaultString(uint8_t fault_code)
{
    static char fault_str[100];
    fault_str[0] = '\0';
    
    if (fault_code == MOTOR_FAULT_NONE) {
        return "No Fault";
    }
    
    if (fault_code & MOTOR_FAULT_SENSOR)      strcat(fault_str, "Sensor ");
    if (fault_code & MOTOR_FAULT_OVERCURRENT) strcat(fault_str, "OverCurrent ");
    if (fault_code & MOTOR_FAULT_PHASE_OC)    strcat(fault_str, "PhaseOC ");
    if (fault_code & MOTOR_FAULT_STALL)       strcat(fault_str, "Stall ");
    if (fault_code & MOTOR_FAULT_OVERTEMP)    strcat(fault_str, "OverTemp ");
    
    return fault_str;
}

// 检查电机是否有故障
uint8_t Motor_HasFault(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return 0;
    return (g_motor_status[motor_index].fault_code != MOTOR_FAULT_NONE);
}

// 清除通信错误计数
void Motor_ClearCommErrorCount(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return;
    g_motor_status[motor_index].comm_error_count = 0;
}





// 电机系统初始化
void Motor_SystemInit(void)
{
    // 清零所有状态
    memset(g_motor_status, 0, sizeof(g_motor_status));
    // 然后设置枚举成员
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        g_motor_status[i].mode = MOTOR_MODE_SPEED;
    }
    
    // 延迟确保电机上电稳定
    HAL_Delay(500);
    
    // 设置所有电机为速度环模式（默认模式）
    Motor_SetModeAll(MOTOR_MODE_SPEED);
    
    // 初始化全局运动控制结构体的电机模式
    g_moving_wheel.MotorMode = MOTOR_SPEED;
    
    // 初始化系统模式为锁定状态（安全起见）
    g_moving_wheel.SystemMode = SYS_LOCKED;
}

// 紧急停止所有电机
void Motor_EmergencyStop(void)
{
    // 发送零速度指令
    int16_t zero_rpm[MOTOR_COUNT] = {0, 0, 0, 0};
    Motor_SendSpeedArray(zero_rpm);
    
    // 清零全局变量
    memset(rpm_motors, 0, sizeof(rpm_motors));
    g_moving_wheel.B_Vel = 0.0f;
    g_moving_wheel.D_Vel = 0.0f;
    g_moving_wheel.F_Vel = 0.0f;
    g_moving_wheel.H_Vel = 0.0f;
}


// 根据g_moving_wheel状态更新电机控制
void Motor_UpdateControl(void)
{
    // 根据系统模式决定电机状态
    switch (g_moving_wheel.SystemMode)
    {
        case SYS_LOCKED:
            // 锁定模式：紧急停止所有电机
            Motor_EmergencyStop();
            break;
            
        case SYS_MOVING:
            // 运动模式：发送速度指令
            {
                int16_t rpm_array[MOTOR_COUNT];
                rpm_array[0] = (int16_t)g_moving_wheel.B_Vel;
                rpm_array[1] = (int16_t)g_moving_wheel.D_Vel;
                rpm_array[2] = (int16_t)g_moving_wheel.F_Vel;
                rpm_array[3] = (int16_t)g_moving_wheel.H_Vel;
                Motor_SendSpeedArray(rpm_array);
            }
            break;
            
        default:
            // 未知状态：安全起见停止电机
            Motor_EmergencyStop();
            break;
    }
}






//驱动行进电机
void MotorDriver_UpdateMotors(MovingWheel_t *moving_wheel)
{
    // 将角速度(°/s)转换为RPM
    // 公式：RPM = (°/s) × 60 / 360 = (°/s) / 6
    int16_t motor_rpm[MOTOR_COUNT];
    
    motor_rpm[0] = (int16_t)(moving_wheel->B_Vel * 60.0f / 360.0f);  // B轮(前左)
    motor_rpm[1] = (int16_t)(moving_wheel->D_Vel * 60.0f / 360.0f);  // D轮(前右)
    motor_rpm[2] = (int16_t)(moving_wheel->F_Vel * 60.0f / 360.0f);  // F轮(后右)
    motor_rpm[3] = (int16_t)(moving_wheel->H_Vel * 60.0f / 360.0f);  // H轮(后左)
    
    // 通过RS485发送给DDSM115电机
    Motor_SendSpeedArray(motor_rpm);
}


//停止所有电机
void MotorDriver_StopAll(void)
{
    // 停止所有驱动电机
    int16_t zero_rpm[MOTOR_COUNT] = {0, 0, 0, 0};
    Motor_SendSpeedArray(zero_rpm);
}





