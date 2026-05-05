/*
 * basic.h (原 math.h)
 *
 * 创建日期: 2025年5月24日
 * 作者: MAO
 * 描述: 专为电力电子与电机控制打造的底层数学运算与坐标变换库。
 * 核心算法全部采用宏定义 (#define) 实现，旨在消除高频中断(如40kHz)下
 * 函数调用的压栈/出栈开销，追求极致的执行速度。
 */

#ifndef LIB_MATH_INCLUDE_BASIC_H_
#define LIB_MATH_INCLUDE_BASIC_H_


// 绝对限幅器：将变量 x 限制在 [min, max] 范围内
#define UPD_LIMIT(x, max, min) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))
// 浮点数绝对值：极速获取正数
#define FLOAT_ABS(x) ((x) >= 0.0f ? (x) : (-(x)))



#define ONE_SQRT3           0.57735026918963f  // 1 / √3 (用于 Clarke 变换)
#define ONE_TWO             0.50f              // 1 / 2
#define ONE_THREE           0.33333333333f     // 1 / 3
#define TWO_THREE           0.66666666667f     // 2 / 3  (用于等幅值 Clarke 变换)
#define SQRT__3_4           0.86602540378444f  // √3 / 2
#define SQRT_2              1.4142135623f      // √2     (用于有效值/峰值换算)
#define DPI                 6.28318530717958f  // 2 * PI (用于频率转角速度)


/* =======================================================================
 * 抗饱和 PI 控制器 (位置式)
 * 公式: u(k) = Kp*e(k) + Ui(k)
 * 特性: 包含“积分分离(Integral separation)”与“反算退饱和(Reverse correction)”
 * ======================================================================= */

// PI 控制器参数配置结构体
typedef struct {
    float  Kp;                // 比例增益 (决定动态响应速度)
    float  Ki;                // 积分增益 (消除稳态误差)
    float  Kc;                // 积分反算校正增益 (决定退出饱和区的速度)
    float  Ts;                // 控制周期 / 采样时间 (如 0.000025f 表示 25us)

    float  outMax;            // 总输出上限
    float  outMin;            // 总输出下限
    float  uiMax;             // 积分器输出上限 (防积分风暴)
    float  uiMin;             // 积分器输出下限
    float  upMax;             // 比例器输出上限
    float  upMin;             // 比例器输出下限
} PI_PARAMS;

// PI 控制器运行状态结构体
typedef struct {
    PI_PARAMS  param;         // 参数组
    float  ref;               // 目标参考值 (期望值)
    float  fb;                // 实际反馈值 (采样值)
    float  err;               // 当前误差 (err = ref - fb)

    float  out;               // 最终受限后的实际输出
    float  outPreSat;         // 饱和前的理论计算输出
    float  outErr;            // 饱和误差 (实际输出 - 理论输出，用于反算补偿)

    float  up;                // 当前周期的比例项输出
    float  ui;                // 当前周期的积分项输出 (不断累加)
} PI_CTRL;

// 核心计算宏：执行一次 PI 迭代计算
#define PI_Calc(v)                                                                     \
do {                                                                                   \
    /* 1. 计算当前跟踪误差 */                                                             \
    v->err = v->ref - v->fb;                                                           \
                                                                                       \
    /* 2. 积分分离逻辑：如果误差极大(超过参考值的110%)，强行将历史积分衰减一半，防止积分超调炸机 */ \
    if( FLOAT_ABS(v->err) > FLOAT_ABS(1.1f * v->ref) )                                 \
    {                                                                                  \
        /* 积分项 = 衰减一半的旧积分 + 饱和反算补偿 */                                       \
        v->ui = 0.5f * v->ui + v->param.Kc * v->outErr;                                \
    } else                                                                             \
    {                                                                                  \
        /* 正常积分累加：旧积分 + Ki*err*Ts + 饱和反算补偿 */                                \
        v->ui += (v->param.Ki * v->err * v->param.Ts + v->param.Kc * v->outErr);       \
    }                                                                                  \
                                                                                       \
    /* 3. 计算比例项并独立限幅 */                                                          \
    v->up = v->param.Kp * v->err;                                                      \
    v->up = UPD_LIMIT(v->up, v->param.upMax, v->param.upMin);                          \
                                                                                       \
    /* 4. 记录饱和前的积分理论值，并对积分项进行安全限幅 */                                     \
    v->outPreSat = v->ui;                                                              \
    v->ui = UPD_LIMIT(v->ui, v->param.uiMax, v->param.uiMin);                          \
                                                                                       \
    /* 5. 计算饱和溢出量(被砍掉的部分)，留给下个周期的 Kc 乘以这个差值来进行反算退饱和 */         \
    v->outErr = v->ui - v->outPreSat;                                                  \
                                                                                       \
    /* 6. 合成最终输出并进行总限幅 */                                                      \
    v->out = UPD_LIMIT(v->up + v->ui, v->param.outMax, v->param.outMin);               \
} while(0)


/* =======================================================================
 * 【一阶数字低通滤波器 (1st-Order Low-Pass Filter)】
 * 原理说明：
 * 连续域传递函数: G(s) = Wc / (s + Wc)，其中 Wc 是截止角频率。
 * 使用“后向欧拉法(Backward Euler method)”进行离散化替换: s = (z - 1) / (z * Ts)
 * 最终推导出时域差分方程: y(k) = k * y(k - 1) + (1 - k) * u(k)
 * ======================================================================= */

// 低通滤波器参数结构体
typedef struct {
    float T;   // 采样周期 (Ts，例如 25us)
    float f_c; // 设定的截止频率 (Hz，例如 10Hz)
    float w_c; // 截止角频率 (w_c = 2 * PI * f_c)
    float k;   // 离散化后的核心滤波系数 (根据 T 和 w_c 计算得出)
} LowPassFilterParams;

// 低通滤波器运行状态结构体
typedef struct {
    LowPassFilterParams params;  // 滤波器参数
    float in;              // 当前输入信号 u(k)
    float out;             // 当前输出信号 y(k)
    float prev_out;        // 上一次的输出信号 y(k-1)
} LowPassFilter;

// 参数更新宏：当采样率或截止频率改变时，重新计算核心滤波系数 k
#define LOW_PASS_FILTER_PARAM_UPDATE(filter)                                                    \
do {                                                                                            \
    /* 计算角频率: w_c = 2 * PI * f_c */                                                          \
    (filter)->params.w_c = __mpy2pif32 ( (filter)->params.f_c );                                \
    /* 计算离散系数: k = 1 / (1 + Ts * w_c)  注：__divf32是DSP内部快速浮点除法指令 */               \
    (filter)->params.k = __divf32( 1.0f , (1.0f + (filter)->params.T * (filter)->params.w_c) ); \
} while (0)

// 核心计算宏：执行一次滤波迭代
#define LOW_PASS_FILTER_CALC(filter)                                                                      \
do {                                                                                                      \
    /* 核心差分方程: 当前输出 = k * 上次输出 + (1 - k) * 当前输入 */                                         \
    (filter)->out = (filter)->params.k * (filter)->prev_out + (1.0f - (filter)->params.k) * (filter)->in; \
    /* 保存当前输出，作为下一次计算的“历史数据” */                                                            \
    (filter)->prev_out = (filter)->out;                                                                   \
} while (0)


/* =======================================================================
 * 【空间矢量坐标变换 (Space Vector Transformations)】
 * 作用：将三相交流正弦变量 (A,B,C) 逐步映射为 直流变量 (D,Q)，以便于 PI 控制器零误差跟踪。
 * ======================================================================= */

// 坐标变换统一定义的结构体 (包含了所有的静止系、旋转系分量和角度)
typedef struct {
    // 三相静止坐标系 (自然坐标)
    float  A;
    float  B;
    float  C;

    // 两相正交静止坐标系
    float  Alpha;
    float  Beta;
    float  Zero;      // 零序分量

    // 矢量幅值与角度 (主要用于极坐标分析，此处预留)
    float  Module;
    float  Angle;

    // 同步旋转坐标系与电网/转子角度
    float  DQ_Angle;      // 当前同步旋转角 θ (通常来自锁相环或编码器)
    float  Cos_DQ_Angle;  // cos(θ)
    float  Sin_DQ_Angle;  // sin(θ)
    float  D;             // 直轴分量 (有功电流/激磁分量)
    float  Q;             // 交轴分量 (无功电流/转矩分量)
} VECTOR;


// 【Clarke 变换】: 三相静止 (A,B,C) -> 两相静止 (Alpha, Beta) (等幅值变换)
#define Clarke(v)                                                       \
do                                                                      \
{                                                                       \
    /* Alpha = 2/3 * (A - 0.5*B - 0.5*C) */                             \
    v->Alpha = (v->A - ONE_TWO * (v->B + v->C )) * TWO_THREE ;          \
    /* Beta = (B - C) / √3 */                                           \
    v->Beta  = (v->B - v->C) * ONE_SQRT3;                               \
    /* 零序分量 = (A + B + C) / 3 */                                      \
    v->Zero  = (v->A + v->B + v->C )  * ONE_THREE;                      \
}while(0)

// 【逆 Clarke 变换】: 两相静止 (Alpha, Beta) -> 三相静止 (A,B,C)
#define IClarke(v)                                                      \
do                                                                      \
{                                                                       \
    /* A = Alpha + Zero */                                              \
    v->A = v->Alpha +  v->Zero;                                         \
    /* B = -0.5*Alpha + (√3/2)*Beta + Zero */                           \
    v->B = (-ONE_TWO * v->Alpha) + (SQRT__3_4 * v->Beta) +  v->Zero;    \
    /* C = -0.5*Alpha - (√3/2)*Beta + Zero */                           \
    v->C = (-ONE_TWO * v->Alpha) - (SQRT__3_4 * v->Beta) +  v->Zero;    \
}while(0)


// 【Park 变换】: 两相静止 (Alpha, Beta) -> 同步旋转 (D, Q)
#define Park(v)                                                         \
do                                                                      \
{                                                                       \
    /* 实时计算角度的三角函数值 (__cos和__sin为DSP内置极速查表指令) */             \
    v->Cos_DQ_Angle =  __cos(v->DQ_Angle);                              \
    v->Sin_DQ_Angle =  __sin(v->DQ_Angle);                              \
    /* D = Alpha*cosθ + Beta*sinθ */                                    \
    v->D = (v->Alpha * v->Cos_DQ_Angle) + (v->Beta  * v->Sin_DQ_Angle); \
    /* Q = Beta*cosθ - Alpha*sinθ */                                    \
    v->Q = (v->Beta  * v->Cos_DQ_Angle) - (v->Alpha * v->Sin_DQ_Angle); \
}while(0)

// 【逆 Park 变换】: 同步旋转 (D, Q) -> 两相静止 (Alpha, Beta)
#define IPark(v)                                                        \
do                                                                      \
{                                                                       \
    v->Cos_DQ_Angle =  __cos(v->DQ_Angle);                              \
    v->Sin_DQ_Angle =  __sin(v->DQ_Angle);                              \
    /* Alpha = D*cosθ - Q*sinθ */                                       \
    v->Alpha = (v->D * v->Cos_DQ_Angle) - (v->Q * v->Sin_DQ_Angle);     \
    /* Beta = Q*cosθ + D*sinθ */                                        \
    v->Beta  = (v->Q * v->Cos_DQ_Angle) + (v->D * v->Sin_DQ_Angle);     \
}while(0)

#endif /* LIB_MATH_INCLUDE_BASIC_H_ */
