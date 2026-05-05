/*
 * sogi_spll.c
 *
 * 创建日期: 2025年7月27日
 * 作者: MAO
 * 描述: 单相锁相环 (SOGI-SPLL) 核心算法模块。
 * 功能包含：基于双线性变换(Tustin)离散化的 SOGI 正交信号发生器 (OSG)、
 * Park 变换、PLL 闭环 PI 控制器（环路滤波器），以及压控振荡器(VCO)的相位积分。
 */

#include <basic.h>
#include <string.h>
#include "sogi_spll.h"
#include "powercalc.h"
#include "parameter.h"
#include "epwm.h"

PLL Sogi_PLL; // 全局锁相环实例

/*
 * 函数名: Init_PLL_Calc
 * 功能:   初始化 SOGI 参数与 PLL 闭环控制器
 * 原理:   在连续域 s 域中，SOGI 本质是一个带通滤波器(Alpha)和一个低通滤波器(Beta)。
 * 这里使用双线性变换 (s = (2/Ts)*(z-1)/(z+1)) 将其转换为数字差分方程系数。
 */
#pragma CODE_SECTION(Init_PLL_Calc,".TI.ramfunc");
void Init_PLL_Calc(PLL *p )
{
    float wTs = 0.0f;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float c3 = 0.0f;


    memset( (void *) (p) , 0 , sizeof( PLL ) );// 清零 PLL 结构体内存，防止野指针或随机脏数据


    p->V_grid_sogi.W_cen = __mpy2pif32( V_FRE_RATED );// 设置 SOGI 中心角频率 (50Hz 对应的角频率 ω = 2*pi*50)
    p->V_grid_sogi.Ts = ISR_TS; // 控制中断周期 (25us)

    // SOGI 差分方程系数计算 (基于双线性离散化推导) SOGI 阻尼系数 k = SQRT_2，此时系统动态响应与抗谐波能力达到最佳平衡
    wTs = p->V_grid_sogi.W_cen * p->V_grid_sogi.Ts; // ω * Ts
    c1 = wTs * wTs;                                 // (ω*Ts)^2
    c2 = 2.0f * SQRT_2 * wTs;                       // 2 * k * ω * Ts (k=√2)
    c3 = __divf32(1.0f , (c1 + c2 + 4.0f) );        // 公共分母倒数

    p->V_grid_sogi.gain_alpha = c2 * c3; // 计算 Alpha(带通) 和 Beta(低通) 前馈输入增益
    p->V_grid_sogi.gain_beta = SQRT_2 * c1 * c3;

    p->V_grid_sogi.coeff_recursive[0] = -(c1 + c1 - 8.0f) * c3;// 计算历史输出状态 (y[n-1], y[n-2]) 的递归反馈系数
    p->V_grid_sogi.coeff_recursive[1] = -(c1 - c2 + 4.0f) * c3;

    p->V_grid_dq_pi.param.Ts = ISR_TS;// 锁相环PI控制器 (环路滤波器 Loop Filter) 初始化
    p->V_grid_dq_pi.param.Kc = 0.33f; // 积分反算抗饱和系数

    // 限制锁相环输出的角频率偏差幅度，防止电网异常时锁相环跑飞
    // 限制范围为：-50Hz 到 +50Hz 的角频率
    p->V_grid_dq_pi.param.outMax = __mpy2pif32( V_FRE_RATED );
    p->V_grid_dq_pi.param.outMin = __mpy2pif32( -1.0f* V_FRE_RATED  );

    // 同步配置 PI 控制器的比例限幅和积分限幅
    p->V_grid_dq_pi.param.upMax = p->V_grid_dq_pi.param.outMax;
    p->V_grid_dq_pi.param.upMin = p->V_grid_dq_pi.param.outMin;
    p->V_grid_dq_pi.param.uiMax = p->V_grid_dq_pi.param.outMax;
    p->V_grid_dq_pi.param.uiMin = p->V_grid_dq_pi.param.outMin;
}

/*
 * 函数名: PLL_Calc
 * 功能:   执行单步 SOGI-SPLL 迭代运算
 * 运行区: RAM 极速运行 (.TI.ramfunc)，通常放在 40kHz 的高频中断中
 */
#pragma CODE_SECTION(PLL_Calc,".TI.ramfunc");
void PLL_Calc( PLL *p , SAMPLE *v )
{
    Uint16 i = 0;

    // 1. 安全防线：如果交流电压发生保护故障（如严重跌落或过压），
    // 立即重置锁相环内部状态，防止积分器累计错误导致恢复并网时相位错乱。
    if(Prot_Info.Err_Flag.V_AC_SW_Err == true)
    {
        Init_PLL_Calc(p);
        return;
    }

    // 2. 滤除直流偏置：将 ADC 原始电压减去 powercalc.c 中算出的直流偏置。
    // 如果不减去直流偏置，这个偏置进入 SOGI 后会在 DQ 轴上产生 50Hz 的严重低频震荡。
    p->V_grid = v->V_AC - Power_Info.V_AC_Harmonic[0];
    p->V_grid_sogi.in = p->V_grid;

    //3. SOGI 核心差分方程运算 (正交信号发生器 OSG)
    // 算出与电网电压同相位的 Alpha 分量 (充当无延时的带通滤波器)
    p->V_grid_sogi.out_alpha = p->V_grid_sogi.gain_alpha * (p->V_grid_sogi.in - p->V_grid_sogi.in_alpha_last_last) +
                               p->V_grid_sogi.coeff_recursive[0] * p->V_grid_sogi.out_alpha_last +
                               p->V_grid_sogi.coeff_recursive[1] * p->V_grid_sogi.out_alpha_last_last;

    // 算出滞后电网电压 90 度的 Beta 分量 (充当 90 度移相的低通滤波器)
    p->V_grid_sogi.out_beta = p->V_grid_sogi.gain_beta * (p->V_grid_sogi.in + 2.0f * p->V_grid_sogi.in_beta_last + p->V_grid_sogi.in_beta_last_last) +
                              p->V_grid_sogi.coeff_recursive[0] * p->V_grid_sogi.out_beta_last +
                              p->V_grid_sogi.coeff_recursive[1] * p->V_grid_sogi.out_beta_last_last;

    //4. 更新历史数据寄存器 (z^-1, z^-2)
    p->V_grid_sogi.in_alpha_last_last = p->V_grid_sogi.in_alpha_last;
    p->V_grid_sogi.in_alpha_last = p->V_grid_sogi.in;
    p->V_grid_sogi.out_alpha_last_last = p->V_grid_sogi.out_alpha_last;
    p->V_grid_sogi.out_alpha_last = p->V_grid_sogi.out_alpha;

    p->V_grid_sogi.in_beta_last_last = p->V_grid_sogi.in_beta_last;
    p->V_grid_sogi.in_beta_last = p->V_grid_sogi.in;
    p->V_grid_sogi.out_beta_last_last = p->V_grid_sogi.out_beta_last;
    p->V_grid_sogi.out_beta_last = p->V_grid_sogi.out_beta;

    //5. Park 变换 (鉴相器 Phase Detector)
    // 将虚拟出来的 Alpha(静止) 和 Beta(静止) 轴投影到旋转的 D,Q 轴上
    p->V_grid_vector.Alpha = p->V_grid_sogi.out_alpha;
    p->V_grid_vector.Beta = p->V_grid_sogi.out_beta;
    Park( (&(p->V_grid_vector)) );

    //6. 锁相环 PI 控制 (环路滤波器)
    // 根据当前电网电压幅值自适应调整 Kp 和 Ki，保证锁相环带宽恒定
    p->V_grid_dq_pi.param.Kp = __divf32( 50.0f , V_AC_RATED);
    p->V_grid_dq_pi.param.Ki = p->V_grid_dq_pi.param.Kp * __mpy2pif32( V_FRE_RATED * ONE_TWO);

    // 将 Q 轴分量作为误差源 (当系统完美锁相时，电压矢量全部落在 D 轴，Q 轴为 0)
    //Kp = K / V_AC_RATED
    //Ki = Kp × 2π × f_bandwidth
    p->V_grid_dq_pi.ref = p->V_grid_vector.Q;
    p->V_grid_dq_pi.fb = 0.0f;
    PI_Calc( (&(p->V_grid_dq_pi)) ); // PI 控制器输出的是角频率的偏差值 (Δω)

    //7. 相位积分生成 (压控振荡器 VCO)
    // 新相位 = 老相位 + (额定角频率 ωn + PI 调节出的角频率偏差 Δω) * 积分时间 Ts
    p->V_grid_vector.DQ_Angle += ( p->V_grid_dq_pi.out + __mpy2pif32( V_FRE_RATED ) ) * ISR_TS;

    // 对 2*pi (DPI) 取余，保证角度始终在 0 ~ 2*pi 之间翻转
    p->V_grid_vector.DQ_Angle = __fmodf(p->V_grid_vector.DQ_Angle , DPI);

    // 提取并记录最终锁定的电网相位和实际电网频率
    p->V_theta = p->V_grid_vector.DQ_Angle;
    p->V_frq = __div2pif32(p->V_grid_dq_pi.out) + V_FRE_RATED; // f = Δω / 2pi + 50Hz

    // ==== 8. 谐波三角函数表预计算 ====
    // 利用 DSP 内置的查表极速指令，一次性算出基波到 9 次谐波的正弦和余弦值
    // 供外环的重复控制器 (RC) 或谐波注入/补偿算法直接取用
    for( i=0; i<10; i++)
    {
        p->V_theta_cos[i] = __cos( p->V_theta * i );
        p->V_theta_sin[i] = __sin( p->V_theta * i );
    }
}

