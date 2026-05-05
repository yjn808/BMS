#ifndef __MOVINGCONTROL_H
#define __MOVINGCONTROL_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "main.h"

// 数学常数定义
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ========== T型轨迹规划器相关定义 ==========

// 辅助宏定义
#define SQ(x) ((x) * (x))

// 轨迹规划参数
#define ANGLE_TRAJ_MAX_VEL      60.0f     // 角度最大速度 60°/s
#define ANGLE_TRAJ_MAX_ACCEL    120.0f    // 角度最大加速度 120°/s²
#define ANGLE_TRAJ_MAX_DECEL    150.0f    // 角度最大减速度 150°/s²

#define SPEED_TRAJ_MAX_VEL      2000.0f   // 速度最大变化率 2000°/s per s
#define SPEED_TRAJ_MAX_ACCEL    800.0f    // 速度最大加速度 800°/s²  
#define SPEED_TRAJ_MAX_DECEL    1200.0f   // 速度最大减速度 1200°/s²

// 辅助函数
float sign_hard(float val);
float copysign_f(float mag, float sign);
float fmax_f(float a, float b);

// T型轨迹规划器结构体
typedef struct {
    // 轨迹参数
    float Ta_, Tv_, Td_, Tf_;    // 加速、匀速、减速、总时间
    float Ar_, Dr_, Vr_;         // 实际加速度、减速度、最大速度(带符号)
    float Xi_, Xf_, Vi_;         // 初始位置、目标位置、初始速度
    float yAccel_;               // 加速阶段结束位置
    
    // 当前状态输出
    float Y_, Yd_, Ydd_;         // 当前位置、速度、加速度
    
    // 运行状态
    uint8_t is_planned;          // 是否已规划
    uint8_t is_finished;         // 是否完成
    
} TrapTraj_t;

// 系统工作模式枚举
typedef enum 
{
    SYS_UNPLUGED = 0,      // 断电/未插电模式：系统未上电，系统关闭状态    
    SYS_LOCKED,            // 锁定模式：系统上电但运动锁定，转向轮呈X型防滑
    SYS_MOVING             // 运动模式：系统正在执行运动指令
} SystemMode_t;

// 运动模式枚举
typedef enum
{
    MOVE_ACKERMANN = 0,    // 阿克曼转向模式
    MOVE_CRABBING = 1,     // 横移模式
    MOVE_ROTATION = 2      // 旋转模式
} MoveMode_t;

// 电机控制模式枚举
typedef enum
{
    MOTOR_CURRENT = 0,     // 电流环模式
    MOTOR_SPEED,           // 速度环模式
    MOTOR_POSITION         // 位置环模式
} MotorControlMode_t;

// MovingWheel结构体(添加轨迹规划字段)
typedef struct MovingWheel_t_Struct
{
    // ========== 系统状态 ==========
    SystemMode_t SystemMode;        // 系统工作模式
    MoveMode_t MoveMode;           // 运动模式（仅在SYS_MOVING时有效）
    MotorControlMode_t MotorMode;  // 电机控制模式
    uint32_t RCIdleTimeMs;         // 遥控器空闲时间计数器(毫秒)
    
    // ========== 轨迹规划状态 ==========
    uint8_t TrajEnabled;           // 轨迹规划启用标志: 1=启用, 0=禁用
    uint32_t TrajStartTime;        // 轨迹开始时间戳(ms)
    
    // ========== 横移模式参数 ==========
    float CB_Direction;            // 横移方向角度(度)
    float CB_Speed;                // 横移速度(RPM)
    float CB_TargetSpeed;          // 横移目标速度(RPM) - 新增
    
    // ========== 双阿克曼模式参数 ==========
    float DA_Angle;                // DA模式转向角度(度)
    float DA_Speed;                // DA模式基准速度(RPM)
    float DA_SpeedRate;            // DA模式速度比例系数
    float DA_TargetAngle;          // DA模式目标角度(度) - 新增
    float DA_TargetSpeed;          // DA模式目标速度(RPM) - 新增
    
    // ========== 转向轮目标角度 ==========
    float A_Dir,                   // A轮目标角度(度) - 前左
          C_Dir,                   // C轮目标角度(度) - 前右
          E_Dir,                   // E轮目标角度(度) - 后右
          G_Dir;                   // G轮目标角度(度) - 后左
    
    // ========== 转向轮反馈角度 ==========
    float A_Dir_FB,                // A轮实际角度(度) - 前左反馈
          C_Dir_FB,                // C轮实际角度(度) - 前右反馈
          E_Dir_FB,                // E轮实际角度(度) - 后右反馈
          G_Dir_FB;                // G轮实际角度(度) - 后左反馈
          
    // ========== 驱动轮目标速度 ==========
    float B_Vel,                   // B轮目标速度(RPM) - 前左
          D_Vel,                   // D轮目标速度(RPM) - 前右
          F_Vel,                   // F轮目标速度(RPM) - 后右
          H_Vel;                   // H轮目标速度(RPM) - 后左
    
    // ========== 驱动轮反馈速度 ==========
    float B_Vel_FB,                // B轮实际速度(RPM) - 前左反馈
          D_Vel_FB,                // D轮实际速度(RPM) - 前右反馈
          F_Vel_FB,                // F轮实际速度(RPM) - 后右反馈
          H_Vel_FB;                // H轮实际速度(RPM) - 后左反馈
          
} MovingWheel_t;

// 全局变量声明
extern MovingWheel_t g_moving_wheel;

// ========== T型轨迹规划函数声明 ==========
void TrapTraj_Init(TrapTraj_t *traj);
uint8_t TrapTraj_PlanTrapezoidal(TrapTraj_t *traj, float Xf, float Xi, float Vi,
                                 float Vmax, float Amax, float Dmax);
void TrapTraj_Eval(TrapTraj_t *traj, float t);
uint8_t TrapTraj_IsFinished(TrapTraj_t *traj);
void TrapTraj_Reset(TrapTraj_t *traj);
float TrapTraj_GetPosition(TrapTraj_t *traj);
float TrapTraj_GetVelocity(TrapTraj_t *traj);

// ========== 轨迹规划系统函数声明 ==========
void TrajPlanning_SystemInit(void);
float TrajPlanning_GetCurrentTime(void);
void TrajPlanning_EnableTrajectory(uint8_t enable);

// ========== 原有函数声明 ==========
void UGVControlInit(void);
void ControlLoop_Init(void);     
void CrabbingMove(void);
void AckermannMove(void);
void RotationMove(void);

#endif


