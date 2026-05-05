#include "RemoteControl.h"

extern uint16_t ch[18];                    // SBUS解析后的通道数组
extern MovingWheel_t g_moving_wheel;       // 底盘控制结构体

RemoteControl RC;

// 静态变量：用于检测数据变化和模式切换
static uint16_t LastCH0, LastCH1;
static uint8_t LastSystemMode, LastMoveMode;
static uint32_t InitStartTime = 0;

// BCH8复位拨杆状态检测专用变量
static uint16_t LastBCH8_Value = BCH8_RESET1;    // 记录上次BCH8位置
static uint8_t BCH8_ResetTriggered = 0;          // 复位触发标志

// 安全失联保护函数
static void RC_EnterFailsafeMode(void)
{
    // 清零所有控制量
    RC.LeftY = 0.0f;
    RC.RightX = 0.0f;
    RC.RCStatic = 1;                    // 设为静止状态
    
    // 强制系统进入安全模式
    RC.SystemMode = 0;                  // 锁定模式
    g_moving_wheel.SystemMode = SYS_LOCKED;
    g_moving_wheel.RCIdleTimeMs = 0;
}

// 模式验证
static uint8_t RC_ValidateMode(uint8_t mode, uint8_t max_value)
{
    return (mode != 0xFF && mode <= max_value) ? mode : 0xFF;
}

// BCH8复位拨杆动作检测函数
static uint8_t RC_DetectResetTrigger(uint16_t current_bch8)
{
    uint8_t reset_triggered = 0;
    
    // 检测拨杆是否从一个有效位置移动到另一个有效位置
    uint8_t last_pos1 = (abs((int)LastBCH8_Value - BCH8_RESET1) <= BCH8_RESET_TOLERANCE);
    uint8_t last_pos2 = (abs((int)LastBCH8_Value - BCH8_RESET2) <= BCH8_RESET_TOLERANCE);
    uint8_t curr_pos1 = (abs((int)current_bch8 - BCH8_RESET1) <= BCH8_RESET_TOLERANCE);
    uint8_t curr_pos2 = (abs((int)current_bch8 - BCH8_RESET2) <= BCH8_RESET_TOLERANCE);
    
    // 检测从位置1切换到位置2，或从位置2切换到位置1
    if ((last_pos1 && curr_pos2) || (last_pos2 && curr_pos1)) {
        reset_triggered = 1;
    }
    
    // 更新历史值
    LastBCH8_Value = current_bch8;
    
    return reset_triggered;
}

// 遥控器安全初始化
void RCInit(void)
{
    InitStartTime = HAL_GetTick();      // 记录初始化开始时间
    
    // 基础状态初始化
    RC.IsRCConnected = 0;               // 假设未连接，等待验证
    RC.RCStatic = 1;                    // 初始静止状态
    RC.LastUpdateTime = InitStartTime;  // 设为初始化时间，避免立即超时
    RC.InitializationComplete = 0;      // 初始化未完成标志
    
    // SBUS数据安全初始化
    RC.CH0_Steering = CH0_STEERING_CENTER;
    RC.CH1_Throttle = CH1_THROTTLE_CENTER;
    RC.CH4_MoveMode = CCH4_MOVE_ACKERMANN;      // 默认阿克曼模式
    RC.CH8_Reset = BCH8_RESET1;                 // 默认复位位置1
    RC.CH9_SystemMode = ACH9_SYSTEM_LOCKED;     // 默认锁定模式
    
    // 归一化数据清零
    RC.LeftY = 0.0f;
    RC.RightX = 0.0f;
   
    // 模式状态安全初始化
    RC.ResetMode = 0;                   // 复位关闭
    RC.SystemMode = 0;                  // 锁定模式
    RC.MoveMode = 0;                    // 阿克曼模式 
    
    // 系统控制结构体同步
    g_moving_wheel.MotorMode = MOTOR_SPEED;
    g_moving_wheel.SystemMode = SYS_LOCKED;
    g_moving_wheel.MoveMode = MOVE_ACKERMANN;
    g_moving_wheel.RCIdleTimeMs = 0;
    
    // 事件标志初始化
    RC.SystemModeChanged = 0;
    RC.MoveModeChanged = 0;
    RC.ResetModeChanged = 0;
    
    // 静态变量初始化
    LastCH0 = CH0_STEERING_CENTER;
    LastCH1 = CH1_THROTTLE_CENTER;
    LastSystemMode = 0;
    LastMoveMode = 0;
    LastBCH8_Value = BCH8_RESET1;       // 初始化BCH8历史值
    BCH8_ResetTriggered = 0;            // 清除复位触发标志
}

// 遥控器数据获取和处理
void RCGetValue(void)
{
    // 检查初始化稳定期
    if(!RC.InitializationComplete) {
        if(HAL_GetTick() - InitStartTime < RC_INIT_STABILIZATION_MS) {
            return; // 稳定期内不处理数据
        }
        RC.InitializationComplete = 1; // 标记初始化完成
    }
    
    // 从SBUS通道数组读取原始数据
    uint16_t new_ch0 = ch[RC_CH_STEERING];              // CH0: 转向
    uint16_t new_ch1 = ch[RC_CH_THROTTLE];              // CH1: 前进后退 
    uint16_t new_CCH4 = ch[RC_CH_MOVE_MODE_SWITCH];     // CCH4: 运动模式
    uint16_t new_BCH8 = ch[RC_CH_RESET_SWITCH];         // BCH8: 刷新复位
    uint16_t new_ACH9 = ch[RC_CH_SYSTEM_SWITCH];        // ACH9: 系统模式

    // 数据有效性检查 - 根据各通道的具体参数范围
    // CH0转向：200-1800，给予50容差
    // CH1油门：200-1760，给予50容差  
    // 开关通道：使用各自的容差范围
    
    if(new_ch0 < (CH0_STEERING_MIN - 50) || new_ch0 > (CH0_STEERING_MAX + 50) ||
       new_ch1 < (CH1_THROTTLE_MIN - 50) || new_ch1 > (CH1_THROTTLE_MAX + 50)) {

        RC_EnterFailsafeMode();
        return;
    }

    // 开关通道有效性检查
    // CCH4: 200±300, 1000±300, 1800±300
    // BCH8: 200±300, 1800±300  
    // ACH9: 200±300, 1800±300
    uint8_t cch4_valid = (abs((int)new_CCH4 - CCH4_MOVE_ACKERMANN) <= CCH4_MOVE_TOLERANCE) ||
                         (abs((int)new_CCH4 - CCH4_MOVE_CRABBING) <= CCH4_MOVE_TOLERANCE) ||
                         (abs((int)new_CCH4 - CCH4_MOVE_ROTATION) <= CCH4_MOVE_TOLERANCE);
    
    uint8_t bch8_valid = (abs((int)new_BCH8 - BCH8_RESET1) <= BCH8_RESET_TOLERANCE) ||
                         (abs((int)new_BCH8 - BCH8_RESET2) <= BCH8_RESET_TOLERANCE);
    
    uint8_t ach9_valid = (abs((int)new_ACH9 - ACH9_SYSTEM_LOCKED) <= ACH9_SYSTEM_TOLERANCE) ||
                         (abs((int)new_ACH9 - ACH9_SYSTEM_MOVING) <= ACH9_SYSTEM_TOLERANCE);
    
    if(!cch4_valid || !bch8_valid || !ach9_valid) {
        // 开关通道位置异常，触发失联保护
        RC_EnterFailsafeMode();
        return;
    }
    
    // 数据有效，更新到RC结构体
    RC.CH0_Steering = new_ch0;
    RC.CH1_Throttle = new_ch1;
    RC.CH4_MoveMode = new_CCH4;
    RC.CH8_Reset = new_BCH8;
    RC.CH9_SystemMode = new_ACH9;
    
    // 摇杆数据归一化处理 
    RC.LeftY = Sbus_MapThrottle(RC.CH1_Throttle);       // 油门控制
    RC.RightX = Sbus_MapSteering(RC.CH0_Steering);      // 转向控制

    // 三段开关状态解析
    uint8_t new_system_mode = Sbus_GetSystemMode(RC.CH9_SystemMode);
    uint8_t new_move_mode = Sbus_GetMoveMode(RC.CH4_MoveMode);
    
    // BCH8复位拨杆动作检测
    BCH8_ResetTriggered = RC_DetectResetTrigger(new_BCH8);
    
    // 模式验证 
    new_system_mode = RC_ValidateMode(new_system_mode, 1);  // 系统模式：0-1 (锁定/运动)
    new_move_mode = RC_ValidateMode(new_move_mode, 2);      // 运动模式：0-2 (阿克曼/蟹行/旋转)
    
    // 检测模式变化事件
    RC.SystemModeChanged = (new_system_mode != LastSystemMode && new_system_mode != 0xFF) ? 1 : 0;
    RC.MoveModeChanged = (new_move_mode != LastMoveMode && new_move_mode != 0xFF) ? 1 : 0;
    RC.ResetModeChanged = BCH8_ResetTriggered;  // 复位变化基于拨杆动作检测
    
    // 模式状态更新
    if(new_system_mode != 0xFF) RC.SystemMode = new_system_mode;
    if(new_move_mode != 0xFF) RC.MoveMode = new_move_mode;
    if(BCH8_ResetTriggered) {
        RC.ResetMode = 1;  // 触发复位

        Motor_SystemInit();           // 重新初始化电机系统
        MWPowerOnReturnAll();         // MW电机回正
        
        // 复位完成后清除标志
        RC.ResetMode = 0;
    }
    
    // 系统模式同步
    if(RC.SystemModeChanged && RC.SystemMode == 1) { // 要求进入运动模式
        // 确保摇杆在中性位置，防止意外运动
        if(fabs(RC.LeftY) < 0.1f && fabs(RC.RightX) < 0.1f) {
            g_moving_wheel.SystemMode = SYS_MOVING; // 允许运动
        } else {
            RC.SystemMode = 0; // 摇杆不在中性位置，保持锁定
            g_moving_wheel.SystemMode = SYS_LOCKED;
        }
    } else {
        // 其他模式变化正常同步
        if(RC.SystemModeChanged) {
            g_moving_wheel.SystemMode = (SystemMode_t)(RC.SystemMode + 1); // 0→LOCKED, 1→MOVING
        }
        if(RC.MoveModeChanged) {
            g_moving_wheel.MoveMode = (MoveMode_t)RC.MoveMode;
        }
    }
    
    // 数据变化检测
    if((abs((int)LastCH0 - (int)RC.CH0_Steering) > RC_DATA_CHANGE_THRESHOLD) ||
       (abs((int)LastCH1 - (int)RC.CH1_Throttle) > RC_DATA_CHANGE_THRESHOLD) ||
       RC.SystemModeChanged || RC.MoveModeChanged || RC.ResetModeChanged)
    {
        RC.RCStatic = 0;                    // 有变化
        g_moving_wheel.RCIdleTimeMs = 0;    // 重置空闲时间
    }
    else
    {
        RC.RCStatic = 1;                    // 静止状态
        if(g_moving_wheel.RCIdleTimeMs < 65535) {
            g_moving_wheel.RCIdleTimeMs++;
        }
    }
    
    // 连接状态更新
    RC.IsRCConnected = 1;
    RC.LastUpdateTime = HAL_GetTick();
    
    // 保存历史值
    LastCH0 = RC.CH0_Steering;
    LastCH1 = RC.CH1_Throttle;
    LastSystemMode = RC.SystemMode;
    LastMoveMode = RC.MoveMode;
}

// 检测遥控器数据是否变化
uint8_t RCIsChanged(void)
{
    return (RC.RCStatic == 0) ? 1 : 0;
}

// 连接状态检查和超时处理
void RCCheckConnection(void)
{
    // 初始化未完成时不检查超时
    if(!RC.InitializationComplete) {
        return;
    }
    
    // 检查是否超时失联
    if(HAL_GetTick() - RC.LastUpdateTime > RC_CONNECTION_TIMEOUT_MS) 
    {
        RC.IsRCConnected = 0;               // 标记为失联
        RC_EnterFailsafeMode();             // 进入失联保护模式
    }
}

// 检查初始化是否完成
uint8_t RCIsInitialized(void)
{
    return RC.InitializationComplete;
}

// 获取当前复位触发状态
uint8_t RC_IsResetTriggered(void)
{
    return BCH8_ResetTriggered;
}

// 手动清除复位触发标志
void RC_ClearResetTrigger(void)
{
    BCH8_ResetTriggered = 0;
}


