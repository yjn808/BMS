/*
 * powercalc.c
 *
 * 创建日期: 2025年5月17日
 * 作者: MAO
 * 描述: 逆变器功率与信号处理模块。
 * 负责在底层高频中断中实时计算交流信号的有效值(RMS)、直流偏置，
 * 以及对直流母线电压/电流进行低通滤波和平滑功率计算。
 */

#include <string.h>
#include "powercalc.h"
#include "epwm.h"


LowPassFilter LPF_V_DC;              // 直流母线电压的低通滤波器
LowPassFilter LPF_I_DC;              // 直流母线电流的低通滤波器

RMS_OFFSET V_AC_RMS_Offset;          // 电网交流电压的 RMS 与偏置计算器
RMS_OFFSET I_Inv_G_RMS_Offset;       // 并网侧(滤波后)交流电流的 RMS 与偏置计算器
RMS_OFFSET I_Inv_L_RMS_Offset;       // 逆变侧(电感上)交流电流的 RMS 与偏置计算器




void Init_Power_Calc(POWER_INFO *p)
{

    memset( (void *) (p) , 0 , sizeof( POWER_INFO ) );// 1. 数据结构体防野指针清零：将算出的功率、有效值等输出结构体强行写 0


    memset( (void *) (&LPF_V_DC) , 0 , sizeof( LowPassFilter ) ); // 2. 将直流电压/电流的低通滤波器内部状态（历史输入、历史输出等）全部清零
    memset( (void *) (&LPF_I_DC) , 0 , sizeof( LowPassFilter ) );

    // 3. 配置直流母线电压低通滤波器的物理参数
    LPF_V_DC.params.T = ISR_TS ;             // T (控制中断采样周期，例如 25us)
    LPF_V_DC.params.f_c = POWERCALC_LPF_F_C; // fc (截止频率，例如 10Hz，用于滤除开关噪声和 100Hz 工频纹波)


    LPF_I_DC.params.T = ISR_TS ;  // 4. 配置直流母线电流低通滤波器的物理参数
    LPF_I_DC.params.f_c = POWERCALC_LPF_F_C;

    //K = 1/(1+Txfs)
    LOW_PASS_FILTER_PARAM_UPDATE( ( &(LPF_V_DC) ) );// 5. 调用底层宏，根据物理参数(T和fc)计算出滤波器的离散化差分系数(k)
    LOW_PASS_FILTER_PARAM_UPDATE( ( &(LPF_I_DC) ) );


    memset( (void *) (&V_AC_RMS_Offset) , 0 , sizeof( RMS_OFFSET ) );//6.将所有交流信号的 RMS 累加器和采样计数器彻底清零，防止开机初始值爆炸
    memset( (void *) (&I_Inv_G_RMS_Offset) , 0 , sizeof( RMS_OFFSET ) );
    memset( (void *) (&I_Inv_L_RMS_Offset) , 0 , sizeof( RMS_OFFSET ) );
}

/*
 * 函数名: Power_Calc
 * 功能:   执行信号处理与功率聚合运算
 * 运行区: 指定在 RAM 中运行 (.TI.ramfunc)，确保在 40kHz 高频中断内的极速执行
 * 参数:   v - 包含最新 ADC 采样值的结构体指针
 * p - 存放计算结果的功率信息结构体指针
 */
#pragma CODE_SECTION(Power_Calc,".TI.ramfunc");
void Power_Calc(SAMPLE *v , POWER_INFO *p)
{
    //第一部分：交流信号特征提取 (有效值与直流分量)
    V_AC_RMS_Offset.data_in = v->V_AC;// 1. 电网电压
    RMS_Offset_cla(&V_AC_RMS_Offset);

    I_Inv_G_RMS_Offset.data_in = v->I_Inv_G;// 2. 网侧并网电流
    RMS_Offset_cla(&I_Inv_G_RMS_Offset);

    I_Inv_L_RMS_Offset.data_in = v->I_Inv_L;// 3. 桥臂电感侧电流
    RMS_Offset_cla(&I_Inv_L_RMS_Offset);

    //第二部分：直流量平滑滤波
    LPF_V_DC.in = v->V_DC;// 将带有高频开关纹波的原始 ADC 采样值喂给滤波器
    LPF_I_DC.in = v->I_DC;

    LOW_PASS_FILTER_CALC( ( &(LPF_V_DC) ) ); // 执行一阶离散低通滤波运算
    LOW_PASS_FILTER_CALC( ( &(LPF_I_DC) ) );

    //第三部分：数据更新与功率计算
    p->V_DC = LPF_V_DC.out;// 获取平滑后的纯净直流基值
    p->I_DC = LPF_I_DC.out;

    p->P_DC = p->V_DC * p->I_DC;// 计算实时直流功率 (P = U * I)

    p->V_AC_RMS = V_AC_RMS_Offset.data_RMS_out;// 获取交流信号的有效值 (每走完 800 个点，即一个工频周期，更新一次)
    p->I_Inv_L_RMS = I_Inv_L_RMS_Offset.data_RMS_out;
    p->I_Inv_G_RMS = I_Inv_G_RMS_Offset.data_RMS_out;

    p->V_AC_Harmonic[0] = V_AC_RMS_Offset.data_OFFSET_out;// 获取交流信号的直流偏置 (即 0 次谐波分量，并网要求直流注入极小)
    p->I_Inv_G_Harmonic[0] = I_Inv_G_RMS_Offset.data_OFFSET_out;
}

/*
 * 函数名: RMS_Offset_cla
 * 功能:   滑动周波计算器 (计算有效值（RMS）和直流偏置（DC Offset）)
 * 运行区: RAM 运行 (.TI.ramfunc)
 * 公式:   RMS = sqrt( (1/N) * Σ(x^2) )
 * Offset = (1/N) * Σ(x)
 */
#pragma CODE_SECTION(RMS_Offset_cla,".TI.ramfunc");
void RMS_Offset_cla(RMS_OFFSET *v)
{

    v->data_RMS_sum += __divf32( (v->data_in * v->data_in) , (float)N );// 1. RMS 平方和累加：采用先除以 N 再累加的方式，即x1^2/800 +x2^2/800 +x3^2/800 +x4^2/800 +...... // 使用 DSP 快速除法指令 __divf32

    v->data_OFFSET_sum += __divf32( v->data_in  , (float)N );// 2. 直流偏置(平均值)累加：同样先除以 N 再累加  即x1/800 +x2/800 +x3/800 +x4/800 +......

    v->count_samp++;// 3. 采样点计数器自增


    if(v->count_samp == N) // 4. 当一个完整的工频周期 (例如 N = 800 个点对应 20ms @ 40kHz)
    {

        v->data_RMS_out = __sqrt(v->data_RMS_sum);// 算出最终的有效值 (对平方和开根号，使用内置快速开方指令 __sqrt)

        v->data_OFFSET_out = v->data_OFFSET_sum;  // 算出最终的直流偏置


        v->count_samp = 0; // 清零计数器与累加器，准备迎接下一个 20ms 周期的计算
        v->data_RMS_sum = 0.0f;
        v->data_OFFSET_sum = 0.0f;
    }
}
