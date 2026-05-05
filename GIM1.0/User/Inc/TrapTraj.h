#ifndef __TRAP_TRAJ_H
#define __TRAP_TRAJ_H

#include <math.h>

// 梯形轨迹规划器配置参数结构体
typedef struct {
    float max_velocity;      // 最大速度
    float max_acceleration;  // 最大加速度
    float max_deceleration; // 最大减速度
} TrapTrajConfig_t;

// 梯形轨迹规划器状态结构体
typedef struct {
    // 轨迹参数
    float Ta_;               // 加速时间
    float Tv_;               // 匀速时间  
    float Td_;               // 减速时间
    float Tf_;               // 总时间
    
    float Ar_;               // 实际加速度(带符号)
    float Dr_;               // 实际减速度(带符号)
    float Vr_;               // 实际最大速度(带符号)
    
    // 初始条件和目标
    float Xi_;               // 初始位置
    float Xf_;               // 目标位置
    float Vi_;               // 初始速度
    float yAccel_;           // 加速阶段结束位置
    
    // 当前状态输出
    float Y_;                // 当前位置
    float Yd_;               // 当前速度
    float Ydd_;              // 当前加速度
    
    // 运行状态
    uint8_t is_planned;      // 是否已规划: 1=已规划, 0=未规划
    uint8_t is_finished;     // 是否完成: 1=完成, 0=运行中
    
} TrapTraj_t;

// 内联辅助函数：符号函数
static inline float sign_hard(float val) {
    return (val < 0.0f) ? -1.0f : 1.0f;
}

// 内联辅助函数：平方
static inline float SQ(float x) {
    return x * x;
}

// 内联辅助函数：最大值
static inline float fmax_f(float a, float b) {
    return (a > b) ? a : b;
}

// 内联辅助函数：最大值(支持0值)
static inline float fmax_zero(float a, float b) {
    return fmax_f(a, b);
}

// 内联辅助函数：符号拷贝
static inline float copysign_f(float mag, float sign) {
    return (sign < 0.0f) ? -fabsf(mag) : fabsf(mag);
}

// ========== 函数声明 ==========

/**
 * @brief 初始化梯形轨迹规划器
 * @param traj 轨迹规划器指针
 */
void TrapTraj_Init(TrapTraj_t *traj);

/**
 * @brief 规划梯形轨迹
 * @param traj 轨迹规划器指针
 * @param Xf 目标位置
 * @param Xi 初始位置  
 * @param Vi 初始速度
 * @param Vmax 最大速度
 * @param Amax 最大加速度
 * @param Dmax 最大减速度
 * @return 1=成功, 0=失败
 */
uint8_t TrapTraj_PlanTrapezoidal(TrapTraj_t *traj, 
                                 float Xf, float Xi, float Vi,
                                 float Vmax, float Amax, float Dmax);

/**
 * @brief 评估轨迹在时间t的状态
 * @param traj 轨迹规划器指针
 * @param t 时间(秒)
 */
void TrapTraj_Eval(TrapTraj_t *traj, float t);

/**
 * @brief 检查轨迹是否完成
 * @param traj 轨迹规划器指针
 * @return 1=完成, 0=未完成
 */
uint8_t TrapTraj_IsFinished(TrapTraj_t *traj);

/**
 * @brief 重置轨迹规划器
 * @param traj 轨迹规划器指针
 */
void TrapTraj_Reset(TrapTraj_t *traj);

/**
 * @brief 获取当前位置
 * @param traj 轨迹规划器指针
 * @return 当前位置
 */
float TrapTraj_GetPosition(TrapTraj_t *traj);

/**
 * @brief 获取当前速度
 * @param traj 轨迹规划器指针
 * @return 当前速度
 */
float TrapTraj_GetVelocity(TrapTraj_t *traj);

/**
 * @brief 获取当前加速度
 * @param traj 轨迹规划器指针
 * @return 当前加速度
 */
float TrapTraj_GetAcceleration(TrapTraj_t *traj);

/**
 * @brief 获取总运行时间
 * @param traj 轨迹规划器指针
 * @return 总时间(秒)
 */
float TrapTraj_GetTotalTime(TrapTraj_t *traj);

// ========== 应用示例宏定义 ==========

// 角度轨迹规划参数(度/秒)
#define ANGLE_TRAJ_MAX_VEL      90.0f     // 最大角速度 90°/s
#define ANGLE_TRAJ_MAX_ACCEL    180.0f    // 最大角加速度 180°/s²
#define ANGLE_TRAJ_MAX_DECEL    180.0f    // 最大角减速度 180°/s²

// 速度轨迹规划参数(RPM/秒) 
#define SPEED_TRAJ_MAX_VEL      1000.0f   // 最大速度变化率 1000RPM/s
#define SPEED_TRAJ_MAX_ACCEL    500.0f    // 最大加速度 500RPM/s²  
#define SPEED_TRAJ_MAX_DECEL    800.0f    // 最大减速度 800RPM/s²

// 位置轨迹规划参数(根据实际需求调整)
#define POS_TRAJ_MAX_VEL        2.0f      // 最大位置速度
#define POS_TRAJ_MAX_ACCEL      1.0f      // 最大位置加速度
#define POS_TRAJ_MAX_DECEL      1.5f      // 最大位置减速度

#endif // __TRAP_TRAJ_H


