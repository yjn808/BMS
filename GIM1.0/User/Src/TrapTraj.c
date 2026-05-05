#include "TrapTraj.h"


//初始化梯形轨迹规划器
void TrapTraj_Init(TrapTraj_t *traj)
{
    if (traj == NULL) return;
    
    // 清零所有状态
    memset(traj, 0, sizeof(TrapTraj_t));
    
    // 初始化标志
    traj->is_planned = 0;
    traj->is_finished = 1;
}

//规划梯形轨迹
uint8_t TrapTraj_PlanTrapezoidal(TrapTraj_t *traj, 
                                 float Xf, float Xi, float Vi,
                                 float Vmax, float Amax, float Dmax)
{
    if (traj == NULL) return 0;
    
    // 参数有效性检查
    if (Vmax <= 0.0f || Amax <= 0.0f || Dmax <= 0.0f) {
        return 0;
    }
    
    float dX = Xf - Xi;  // 距离差
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // 最小停止距离
    float dXstop = copysign_f(stop_dist, Vi); // 最小停止位移(带符号)
    float s = sign_hard(dX - dXstop); // 海岸速度的符号(如果有的话)
    
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
    // 获得到达巡航速度所需的最小位移
    float dXmin = 0.5f * traj->Ta_ * (traj->Vr_ + Vi) + 0.5f * traj->Td_ * traj->Vr_;
    
    // 检查位移是否足够达到巡航速度
    if (s * dX < s * dXmin)
    {
        // 短距离移动(三角形轮廓)
        float temp = (traj->Dr_ * SQ(Vi) + 2 * traj->Ar_ * traj->Dr_ * dX) / (traj->Dr_ - traj->Ar_);
        traj->Vr_ = s * sqrtf(fmax_zero(temp, 0.0f));
        traj->Ta_ = fmax_zero((traj->Vr_ - Vi) / traj->Ar_, 0.0f);
        traj->Td_ = fmax_zero(-traj->Vr_ / traj->Dr_, 0.0f);
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
    traj->yAccel_ = Xi + Vi * traj->Ta_ + 0.5f * traj->Ar_ * SQ(traj->Ta_); // 加速阶段结束位置
    
    // 设置状态标志
    traj->is_planned = 1;
    traj->is_finished = 0;
    
    return 1;
}

//评估轨迹在时间t的状态
void TrapTraj_Eval(TrapTraj_t *traj, float t)
{
    if (traj == NULL || !traj->is_planned) return;
    
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

//检查轨迹是否完成
uint8_t TrapTraj_IsFinished(TrapTraj_t *traj)
{
    if (traj == NULL) return 1;
    return traj->is_finished;
}

//重置轨迹规划器
void TrapTraj_Reset(TrapTraj_t *traj)
{
    if (traj == NULL) return;
    
    traj->is_planned = 0;
    traj->is_finished = 1;
    traj->Y_ = 0.0f;
    traj->Yd_ = 0.0f;
    traj->Ydd_ = 0.0f;
}

//获取当前位置 
float TrapTraj_GetPosition(TrapTraj_t *traj)
{
    if (traj == NULL) return 0.0f;
    return traj->Y_;
}

//获取当前速度
float TrapTraj_GetVelocity(TrapTraj_t *traj)
{
    if (traj == NULL) return 0.0f;
    return traj->Yd_;
}

//获取当前加速度
float TrapTraj_GetAcceleration(TrapTraj_t *traj)
{
    if (traj == NULL) return 0.0f;
    return traj->Ydd_;
}

//获取总运行时间
float TrapTraj_GetTotalTime(TrapTraj_t *traj)
{
    if (traj == NULL || !traj->is_planned) return 0.0f;
    return traj->Tf_;
}


