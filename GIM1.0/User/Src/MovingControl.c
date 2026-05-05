#include "MovingControl.h"


// 机器人物理参数定义
#define CARWIDTH						0.4f          // 车宽：40cm
#define CARLENGTH						0.5f        // 车长：50cm
#define DA_MAXANGLE 				(PI-atan2(CARLENGTH,CARWIDTH))/PI*180.0f  // 双阿克曼模式最大转向角（约118°）

#define CONTROL_FREQ 50
#define CONTROL_PERIOD 1000000/CONTROL_FREQ-1

#define MAX_SPEED		0.4f/0.05f*360.0f // 最大速度：约2880°/s
#define MAX_ACCELERATION  3.0f/0.05f*360.0f/CONTROL_FREQ // 最大加速度（受控制频率限制）

MovingWheel_t g_moving_wheel;

//T型轨迹规划器
static TrapTraj_t angle_trajectory;     // 角度轨迹规划器(阿克曼转向角)
static TrapTraj_t speed_trajectory;     // 速度轨迹规划器(前进后退速度)
static TrapTraj_t crab_speed_trajectory; // 蟹行速度轨迹规划器

extern RemoteControl RC;

// ========== 辅助函数实现 ==========
float sign_hard(float val)
{
    return (val < 0.0f) ? -1.0f : 1.0f;
}

float copysign_f(float mag, float sign)
{
    return (sign < 0.0f) ? -fabsf(mag) : fabsf(mag);
}

float fmax_f(float a, float b)
{
    return (a > b) ? a : b;
}

// ========== T型轨迹规划器核心实现 ==========
void TrapTraj_Init(TrapTraj_t *traj)
{
    if (traj == ((void*)0)) return;
    memset(traj, 0, sizeof(TrapTraj_t));
    traj->is_planned = 0;
    traj->is_finished = 1;
}

uint8_t TrapTraj_PlanTrapezoidal(TrapTraj_t *traj, float Xf, float Xi, float Vi,
                                 float Vmax, float Amax, float Dmax)
{
    if (traj == ((void*)0) || Vmax <= 0.0f || Amax <= 0.0f || Dmax <= 0.0f) {
        return 0;
    }
    
    float dX = Xf - Xi;  // 距离差
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // 最小停止距离
    float dXstop = copysign_f(stop_dist, Vi); // 最小停止位移(带符号)
    float s = sign_hard(dX - dXstop); // 海岸速度的符号
    
    // 设置带符号的运动学参数
    traj->Ar_ = s * Amax;  // 最大加速度(带符号)
    traj->Dr_ = -s * Dmax; // 最大减速度(带符号)
    traj->Vr_ = s * Vmax;  // 最大速度(带符号)
    
    // 如果初始速度比巡航速度快，需要减速而不是加速
    if ((s * Vi) > (s * traj->Vr_))
    {
        traj->Ar_ = -s * Amax;
    }
    
    // 加速/减速到巡航速度的时间
    traj->Ta_ = (traj->Vr_ - Vi) / traj->Ar_;
    traj->Td_ = -traj->Vr_ / traj->Dr_;
    
    // 在完整加速和减速时间内速度斜坡的积分
    float dXmin = 0.5f * traj->Ta_ * (traj->Vr_ + Vi) + 0.5f * traj->Td_ * traj->Vr_;
    
    // 检查位移是否足够达到巡航速度
    if (s * dX < s * dXmin)
    {
        // 短距离移动(三角形轮廓)
        float temp = (traj->Dr_ * SQ(Vi) + 2 * traj->Ar_ * traj->Dr_ * dX) / (traj->Dr_ - traj->Ar_);
        traj->Vr_ = s * sqrtf(fmax_f(temp, 0.0f));
        traj->Ta_ = fmax_f((traj->Vr_ - Vi) / traj->Ar_, 0.0f);
        traj->Td_ = fmax_f(-traj->Vr_ / traj->Dr_, 0.0f);
        traj->Tv_ = 0.0f;
    }
    else
    {
        // 长距离移动(梯形轮廓)
        traj->Tv_ = (dX - dXmin) / traj->Vr_;
    }
    
    // 填写评估时使用的其余值
    traj->Tf_ = traj->Ta_ + traj->Tv_ + traj->Td_;
    traj->Xi_ = Xi;
    traj->Xf_ = Xf;
    traj->Vi_ = Vi;
    traj->yAccel_ = Xi + Vi * traj->Ta_ + 0.5f * traj->Ar_ * SQ(traj->Ta_);
    
    // 设置状态标志
    traj->is_planned = 1;
    traj->is_finished = 0;
    
    return 1;
}

void TrapTraj_Eval(TrapTraj_t *traj, float t)
{
    if (traj == ((void*)0) || !traj->is_planned) return;
    
    if (t < 0.0f)    // 初始条件
    {
        traj->Y_   = traj->Xi_;
        traj->Yd_  = traj->Vi_;
        traj->Ydd_ = 0.0f;
    }
    else if (t < traj->Ta_)      // 加速阶段
    {
        traj->Y_   = traj->Xi_ + traj->Vi_ * t + 0.5f * traj->Ar_ * SQ(t);
        traj->Yd_  = traj->Vi_ + traj->Ar_ * t;
        traj->Ydd_ = traj->Ar_;
    }
    else if (t < traj->Ta_ + traj->Tv_)      // 匀速阶段
    {
        traj->Y_   = traj->yAccel_ + traj->Vr_ * (t - traj->Ta_);
        traj->Yd_  = traj->Vr_;
        traj->Ydd_ = 0.0f;
    }
    else if (t < traj->Tf_)      // 减速阶段
    {
        float td   = t - traj->Tf_;
        traj->Y_   = traj->Xf_ + 0.5f * traj->Dr_ * SQ(td);
        traj->Yd_  = traj->Dr_ * td;
        traj->Ydd_ = traj->Dr_;
    }
    else if (t >= traj->Tf_)      // 最终条件
    {
        traj->Y_   = traj->Xf_;
        traj->Yd_  = 0.0f;
        traj->Ydd_ = 0.0f;
        traj->is_finished = 1;
    }
}

uint8_t TrapTraj_IsFinished(TrapTraj_t *traj)
{
    if (traj == ((void*)0)) return 1;
    return traj->is_finished;
}

void TrapTraj_Reset(TrapTraj_t *traj)
{
    if (traj == ((void*)0)) return;
    traj->is_planned = 0;
    traj->is_finished = 1;
    traj->Y_ = 0.0f;
    traj->Yd_ = 0.0f;
    traj->Ydd_ = 0.0f;
}

float TrapTraj_GetPosition(TrapTraj_t *traj)
{
    if (traj == ((void*)0)) return 0.0f;
    return traj->Y_;
}

float TrapTraj_GetVelocity(TrapTraj_t *traj)
{
    if (traj == ((void*)0)) return 0.0f;
    return traj->Yd_;
}

// ========== 轨迹规划系统管理函数 ==========
void TrajPlanning_SystemInit(void)
{
    // 初始化所有轨迹规划器
    TrapTraj_Init(&angle_trajectory);
    TrapTraj_Init(&speed_trajectory);
    TrapTraj_Init(&crab_speed_trajectory);
    
    // 初始化轨迹规划相关状态
    g_moving_wheel.TrajEnabled = 1;  // 默认启用轨迹规划
    g_moving_wheel.TrajStartTime = HAL_GetTick();
}

float TrajPlanning_GetCurrentTime(void)
{
    return (HAL_GetTick() - g_moving_wheel.TrajStartTime) / 1000.0f; // 转换为秒
}

void TrajPlanning_EnableTrajectory(uint8_t enable)
{
    g_moving_wheel.TrajEnabled = enable;
    
    if (!enable) {
        // 禁用时重置所有轨迹规划器
        TrapTraj_Reset(&angle_trajectory);
        TrapTraj_Reset(&speed_trajectory);
        TrapTraj_Reset(&crab_speed_trajectory);
    }
}

//系统初始化
void UGVControlInit(void)
{
    g_moving_wheel.SystemMode = SYS_UNPLUGED;      // 初始状态：未插电/未连接（安全状态）
    g_moving_wheel.RCIdleTimeMs = 0;               // 遥控器空闲时间清零（失联保护计时器）
    
    // 初始化轨迹规划系统
    TrajPlanning_SystemInit();
}

//主控制循环
void ControlLoop_Init(void)
{
    switch (g_moving_wheel.SystemMode)
    {
        case SYS_LOCKED:
            // 锁定模式：转向轮保持X型，停止驱动
            MW_SetXLock();
            // 停止所有轨迹规划
            TrapTraj_Reset(&angle_trajectory);
            TrapTraj_Reset(&speed_trajectory);
            TrapTraj_Reset(&crab_speed_trajectory);
            break;
            
        case SYS_MOVING:
            // 运动模式
            if (g_moving_wheel.MoveMode == MOVE_ACKERMANN) {
                AckermannMove();
            } else if (g_moving_wheel.MoveMode == MOVE_CRABBING) {
                CrabbingMove();
            } else if (g_moving_wheel.MoveMode == MOVE_ROTATION) {
                RotationMove();
            }
            
            // 更新电机状态
            MotorDriver_UpdateMW(&g_moving_wheel);
            MotorDriver_UpdateMotors(&g_moving_wheel);
            break;
            
        default:
            // 未知状态：安全停止
            Motor_EmergencyStop();
            break;
    }
}

//蟹行模式
void CrabbingMove(void)
{
    static uint8_t crabbing_init_done = 0;
    static uint32_t init_start_time = 0;
    float crab_angle_units = 90.0f / ANGLE_UNIT;  // 90°转换为2.0单位
    
    // 蟹行模式初始化
    if (!crabbing_init_done) {
        if (init_start_time == 0) {
            init_start_time = HAL_GetTick();
            
            // 所有转向轮都转到90°（竖直方向） - 直接控制MW电机
            float target_A = MW_POWERON_RETURN_ANGEL_A + crab_angle_units;  // A轮 +90°
            float target_C = MW_POWERON_RETURN_ANGEL_C + crab_angle_units;  // C轮 +90°
            float target_E = MW_POWERON_RETURN_ANGEL_E + crab_angle_units;  // E轮 +90°
            float target_G = MW_POWERON_RETURN_ANGEL_G + crab_angle_units;  // G轮 +90°
            
            // 直接发送给MW电机
            MWPosControl(1, 1, target_A, 0, 0);  // A轮
            MWPosControl(1, 2, target_C, 0, 0);  // C轮
            MWPosControl(1, 3, target_E, 0, 0);  // E轮
            MWPosControl(1, 4, target_G, 0, 0);  // G轮
            
            // 停止所有驱动轮
            g_moving_wheel.B_Vel = 0.0f;
            g_moving_wheel.D_Vel = 0.0f;
            g_moving_wheel.F_Vel = 0.0f;
            g_moving_wheel.H_Vel = 0.0f;
            
            // 重置横移参数
            g_moving_wheel.CB_Speed = 0.0f;
            g_moving_wheel.CB_TargetSpeed = 0.0f;
        }
        
        // 等待MW电机转到位（1.5秒）并确保摇杆在中性位置
        if (HAL_GetTick() - init_start_time >= 1500) {
            if (fabsf(RC.LeftY) < 0.1f && fabsf(RC.RightX) < 0.1f ) {
                crabbing_init_done = 1;  // 初始化完成
            }
        }
        return;
    }
    
    //带轨迹规划的速度控制
    float target_speed = RC.LeftY * MAX_SPEED;
    
    // 检查目标速度是否改变
    if (fabsf(target_speed - g_moving_wheel.CB_TargetSpeed) > 50.0f) // 50°/s阈值
    {
        if (g_moving_wheel.TrajEnabled) {
            // 使用轨迹规划
            uint8_t plan_result = TrapTraj_PlanTrapezoidal(&crab_speed_trajectory,
                target_speed,           // 目标速度
                g_moving_wheel.CB_Speed,// 当前速度
                0.0f,                   // 当前加速度
                SPEED_TRAJ_MAX_VEL,     // 最大速度变化率
                SPEED_TRAJ_MAX_ACCEL,   // 最大加速度
                SPEED_TRAJ_MAX_DECEL    // 最大减速度
            );
            
            if (plan_result) {
                g_moving_wheel.TrajStartTime = HAL_GetTick();
                g_moving_wheel.CB_TargetSpeed = target_speed;
            }
        } else {
            // 不使用轨迹规划，保持原有逻辑
            g_moving_wheel.CB_TargetSpeed = target_speed;
        }
    }
    
    // 更新速度
    if (g_moving_wheel.TrajEnabled && crab_speed_trajectory.is_planned) {
        float current_time = TrajPlanning_GetCurrentTime();
        TrapTraj_Eval(&crab_speed_trajectory, current_time);
        g_moving_wheel.CB_Speed = TrapTraj_GetPosition(&crab_speed_trajectory);
    } else {
        // 原有加速度限制逻辑
        float SPD = g_moving_wheel.CB_TargetSpeed;
        if (SPD < g_moving_wheel.CB_Speed + MAX_ACCELERATION && 
            SPD > g_moving_wheel.CB_Speed - MAX_ACCELERATION) {
            g_moving_wheel.CB_Speed = SPD;
        } 
        else if (SPD > g_moving_wheel.CB_Speed + MAX_ACCELERATION) {
            g_moving_wheel.CB_Speed = g_moving_wheel.CB_Speed + MAX_ACCELERATION;
        } 
        else if (SPD < g_moving_wheel.CB_Speed - MAX_ACCELERATION) {
            g_moving_wheel.CB_Speed = g_moving_wheel.CB_Speed - MAX_ACCELERATION;
        }
    }
    
    // 3. 驱动轮速度分配：所有轮子同向转动实现前后移动
    g_moving_wheel.B_Vel = -g_moving_wheel.CB_Speed;   // 前左驱动
    g_moving_wheel.D_Vel = -g_moving_wheel.CB_Speed;   // 前右驱动
    g_moving_wheel.F_Vel =  g_moving_wheel.CB_Speed;   // 后右驱动
    g_moving_wheel.H_Vel =  g_moving_wheel.CB_Speed;   // 后左驱动
}

//双阿克曼模式
void AckermannMove(void)
{
    static uint8_t ackermann_init_done = 0;
    static uint32_t init_start_time = 0;

    // 阿克曼模式初始化
    if (!ackermann_init_done) {
        if (init_start_time == 0) {
            init_start_time = HAL_GetTick();
            
            // 强制所有转向轮回到0°（直行位置）- 直接控制MW电机
            MWPosControl(1, 1, MW_POWERON_RETURN_ANGEL_A, 0, 0);  // A轮回正
            MWPosControl(1, 2, MW_POWERON_RETURN_ANGEL_C, 0, 0);  // C轮回正
            MWPosControl(1, 3, MW_POWERON_RETURN_ANGEL_E, 0, 0);  // E轮回正
            MWPosControl(1, 4, MW_POWERON_RETURN_ANGEL_G, 0, 0);  // G轮回正
            
            // 停止所有驱动轮（安全）
            g_moving_wheel.B_Vel = 0.0f;
            g_moving_wheel.D_Vel = 0.0f;
            g_moving_wheel.F_Vel = 0.0f;
            g_moving_wheel.H_Vel = 0.0f;
            
            // 重置阿克曼参数
            g_moving_wheel.DA_Angle = 0.0f;
            g_moving_wheel.DA_Speed = 0.0f;
            g_moving_wheel.DA_SpeedRate = 1.0f;
            g_moving_wheel.DA_TargetAngle = 0.0f;
            g_moving_wheel.DA_TargetSpeed = 0.0f;
        }
        
        if (HAL_GetTick() - init_start_time >= 1000) {
            if (fabsf(RC.LeftY) < 0.1f && fabsf(RC.RightX) < 0.1f) {
                ackermann_init_done = 1;  // 初始化完成
            }
        }
        return;
    }

    //带轨迹规划的角度控制
    #define MAX_ACKERMANN_ANGLE 45.0f
    
    float target_angle = RC.RightX * MAX_ACKERMANN_ANGLE;
    
    // 检查转向角目标是否改变
    if (fabsf(target_angle - g_moving_wheel.DA_TargetAngle) > 1.0f) // 1度阈值
    {
        if (g_moving_wheel.TrajEnabled) {
            // 使用轨迹规划
            uint8_t plan_result = TrapTraj_PlanTrapezoidal(&angle_trajectory,
                target_angle,           // 目标角度
                g_moving_wheel.DA_Angle,// 当前角度
                0.0f,                   // 当前角速度
                ANGLE_TRAJ_MAX_VEL,     // 最大角速度
                ANGLE_TRAJ_MAX_ACCEL,   // 最大角加速度
                ANGLE_TRAJ_MAX_DECEL    // 最大角减速度
            );
            
            if (plan_result) {
                g_moving_wheel.TrajStartTime = HAL_GetTick();
                g_moving_wheel.DA_TargetAngle = target_angle;
            }
        } else {
            // 不使用轨迹规划，直接设置
            g_moving_wheel.DA_TargetAngle = target_angle;
        }
    }
    
    // 更新转向角
    if (g_moving_wheel.TrajEnabled && angle_trajectory.is_planned) {
        float current_time = TrajPlanning_GetCurrentTime();
        TrapTraj_Eval(&angle_trajectory, current_time);
        g_moving_wheel.DA_Angle = TrapTraj_GetPosition(&angle_trajectory);
    } else {
        // 直接设置角度(原有逻辑)
        g_moving_wheel.DA_Angle = g_moving_wheel.DA_TargetAngle;
    }
    
    // 安全限制
    if (g_moving_wheel.DA_Angle > MAX_ACKERMANN_ANGLE)  g_moving_wheel.DA_Angle = MAX_ACKERMANN_ANGLE;
    if (g_moving_wheel.DA_Angle < -MAX_ACKERMANN_ANGLE) g_moving_wheel.DA_Angle = -MAX_ACKERMANN_ANGLE;

    //带轨迹规划的速度控制
    float target_speed = RC.LeftY * MAX_SPEED;
    
    // 检查速度目标是否改变
    if (fabsf(target_speed - g_moving_wheel.DA_TargetSpeed) > 50.0f) // 50°/s阈值
    {
        if (g_moving_wheel.TrajEnabled) {
            // 使用轨迹规划
            uint8_t plan_result = TrapTraj_PlanTrapezoidal(&speed_trajectory,
                target_speed,           // 目标速度
                g_moving_wheel.DA_Speed,// 当前速度
                0.0f,                   // 当前加速度
                SPEED_TRAJ_MAX_VEL,     // 最大速度变化率
                SPEED_TRAJ_MAX_ACCEL,   // 最大加速度
                SPEED_TRAJ_MAX_DECEL    // 最大减速度
            );
            
            if (plan_result) {
                g_moving_wheel.TrajStartTime = HAL_GetTick();
                g_moving_wheel.DA_TargetSpeed = target_speed;
            }
        } else {
            g_moving_wheel.DA_TargetSpeed = target_speed;
        }
    }
    
    // 更新速度
    if (g_moving_wheel.TrajEnabled && speed_trajectory.is_planned) {
        float current_time = TrajPlanning_GetCurrentTime();
        TrapTraj_Eval(&speed_trajectory, current_time);
        g_moving_wheel.DA_Speed = TrapTraj_GetPosition(&speed_trajectory);
    } else {
        // 原有加速度限制逻辑
        float SPD = g_moving_wheel.DA_TargetSpeed;
        if (SPD < g_moving_wheel.DA_Speed + MAX_ACCELERATION && 
            SPD > g_moving_wheel.DA_Speed - MAX_ACCELERATION) {
            g_moving_wheel.DA_Speed = SPD;
        } 
        else if (SPD > g_moving_wheel.DA_Speed + MAX_ACCELERATION) {
            g_moving_wheel.DA_Speed = g_moving_wheel.DA_Speed + MAX_ACCELERATION;
        } 
        else if (SPD < g_moving_wheel.DA_Speed - MAX_ACCELERATION) {
            g_moving_wheel.DA_Speed = g_moving_wheel.DA_Speed - MAX_ACCELERATION;
        }
    }

    //阿克曼转向几何学解算
    float front_inner_angle, front_outer_angle, rear_inner_angle, rear_outer_angle;
    float A_angle_offset, C_angle_offset, E_angle_offset, G_angle_offset;

    if (fabsf(g_moving_wheel.DA_Angle) > 0.1f) {
        // 计算转弯半径
        float R = CARLENGTH / tanf(fabsf(g_moving_wheel.DA_Angle) / 180.0f * PI);
        
        // 前轮阿克曼角度计算
        float R_inner = R - CARWIDTH / 2;  // 内侧转弯半径
        float R_outer = R + CARWIDTH / 2;  // 外侧转弯半径
        
        front_inner_angle = atanf(CARLENGTH / R_inner) * 180.0f / PI;
        front_outer_angle = atanf(CARLENGTH / R_outer) * 180.0f / PI;
        
        // 后轮角度：通常为前轮角度的一半或按几何关系计算
        rear_inner_angle = atanf((CARLENGTH/2) / R_inner) * 180.0f / PI;
        rear_outer_angle = atanf((CARLENGTH/2) / R_outer) * 180.0f / PI;
        
        if (g_moving_wheel.DA_Angle > 0) {
            // 右转：A外C内，E外G内
            A_angle_offset = front_outer_angle;   // 前右外轮
            C_angle_offset = front_inner_angle;   // 前左内轮  
            E_angle_offset = rear_outer_angle;    // 后右外轮
            G_angle_offset = rear_inner_angle;    // 后左内轮
        } else {
            // 左转：A内C外，E内G外
            A_angle_offset = -front_inner_angle;  // 前右内轮
            C_angle_offset = -front_outer_angle;  // 前左外轮
            E_angle_offset = -rear_inner_angle;   // 后右内轮
            G_angle_offset = -rear_outer_angle;   // 后左外轮
        }
    } else {
        // 直行
        A_angle_offset = C_angle_offset = E_angle_offset = G_angle_offset = 0.0f;
    }
    
    //计算最终目标位置并控制MW电机
    // 回正位置 + 偏移量
    float target_A = MW_POWERON_RETURN_ANGEL_A + (A_angle_offset / ANGLE_UNIT);
    float target_C = MW_POWERON_RETURN_ANGEL_C + (C_angle_offset / ANGLE_UNIT);
    float target_E = MW_POWERON_RETURN_ANGEL_E + (E_angle_offset / ANGLE_UNIT);
    float target_G = MW_POWERON_RETURN_ANGEL_G + (G_angle_offset / ANGLE_UNIT);
    
    MWPosControl(1, 1, target_A, 0, 0);  // A轮
    MWPosControl(1, 2, target_C, 0, 0);  // C轮
    MWPosControl(1, 3, target_E, 0, 0);  // E轮
    MWPosControl(1, 4, target_G, 0, 0);  // G轮
    
    //计算内外轮速度比
    if (fabsf(G_angle_offset) > 0.1f) {  // 改进：使用fabsf避免除零
        g_moving_wheel.DA_SpeedRate = 
            sinf(A_angle_offset / 180.0f * PI) / 
            sinf(G_angle_offset / 180.0f * PI);
    } 
    else {
        g_moving_wheel.DA_SpeedRate = 1.0f;
    }
    
    //根据转向方向分配驱动轮速度
    if (g_moving_wheel.DA_Angle > 0) {
        // 右转时
        g_moving_wheel.B_Vel = -g_moving_wheel.DA_Speed;                        
        g_moving_wheel.D_Vel = g_moving_wheel.DA_Speed;                         
        g_moving_wheel.F_Vel = -g_moving_wheel.DA_Speed * g_moving_wheel.DA_SpeedRate; 
        g_moving_wheel.H_Vel = g_moving_wheel.DA_Speed * g_moving_wheel.DA_SpeedRate;  
    } 
    else {
        // 左转时
        g_moving_wheel.B_Vel = -g_moving_wheel.DA_Speed / g_moving_wheel.DA_SpeedRate; 
        g_moving_wheel.D_Vel = g_moving_wheel.DA_Speed / g_moving_wheel.DA_SpeedRate;  
        g_moving_wheel.F_Vel = -g_moving_wheel.DA_Speed;                              
        g_moving_wheel.H_Vel = g_moving_wheel.DA_Speed;                               
    }
}

//旋转模式
void RotationMove(void)
{
    float SPD;  // 目标速度变量
    float rotation_angle_units = 45.0f / ANGLE_UNIT;  // 45°转换为1.0单位
    
    // 旋转模式初始化
    static uint8_t rotation_init_done = 0;
    static uint32_t init_start_time = 0;
    
    if (!rotation_init_done) {
        if (init_start_time == 0) {
            init_start_time = HAL_GetTick();
            
            // 停止所有驱动轮（安全）
            g_moving_wheel.B_Vel = 0.0f;
            g_moving_wheel.D_Vel = 0.0f;
            g_moving_wheel.F_Vel = 0.0f;
            g_moving_wheel.H_Vel = 0.0f;
        }
        
        // 设置转向轮为X型锁定角度（回正位置+偏移量）
        float target_A = MW_POWERON_RETURN_ANGEL_A + rotation_angle_units;  // A轮：回正+45°
        float target_C = MW_POWERON_RETURN_ANGEL_C - rotation_angle_units;  // C轮：回正-45°
        float target_E = MW_POWERON_RETURN_ANGEL_E + rotation_angle_units;  // E轮：回正+45°
        float target_G = MW_POWERON_RETURN_ANGEL_G - rotation_angle_units;  // G轮：回正-45°
        
        MWPosControl(1, 1, target_A, 0, 0);  // A轮
        MWPosControl(1, 2, target_C, 0, 0);  // C轮
        MWPosControl(1, 3, target_E, 0, 0);  // E轮
        MWPosControl(1, 4, target_G, 0, 0);  // G轮
        
        // 等待MW电机转到位（1.5秒）并确保摇杆在中性位置
        if (HAL_GetTick() - init_start_time >= 1500) {
            if (fabsf(RC.LeftY) < 0.1f && fabsf(RC.RightX) < 0.1f) {
                rotation_init_done = 1;  // 初始化完成
            }
        }
        return;
    }
    
    // 速度控制：使用左摇杆Y轴控制旋转速度
    SPD = RC.LeftY * MAX_SPEED * 0.5f;  // 降低旋转速度（50%），避免过快
    
    // 旋转方向控制：使用右摇杆X轴控制旋转方向
    float rotation_direction = 0.0f;
    if (RC.RightX > 0.1f) {
        rotation_direction = 1.0f;   // 顺时针
    } else if (RC.RightX < -0.1f) {
        rotation_direction = -1.0f;  // 逆时针
    }
    
    // 驱动轮速度分配：实现原地旋转
    float rotation_speed = SPD * rotation_direction;
    
    if (rotation_direction != 0.0f) {
        // 所有驱动轮同向转动实现旋转
        g_moving_wheel.B_Vel = rotation_speed;   // 前左驱动
        g_moving_wheel.D_Vel = rotation_speed;   // 前右驱动
        g_moving_wheel.F_Vel = rotation_speed;   // 后右驱动
        g_moving_wheel.H_Vel = rotation_speed;   // 后左驱动
    } else {
        // 停止旋转
        g_moving_wheel.B_Vel = 0.0f;
        g_moving_wheel.D_Vel = 0.0f;
        g_moving_wheel.F_Vel = 0.0f;
        g_moving_wheel.H_Vel = 0.0f;
    }
}

