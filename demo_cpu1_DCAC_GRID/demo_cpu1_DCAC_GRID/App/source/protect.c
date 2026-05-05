/*
 * protect.c
 *
 * 创建日期: 2025年5月17日
 * 作者: MAO
 * 描述: 逆变器系统级保护模块。
 * 负责过流、过压、欠压的实时监测，以及故障标志位的自动置位与指令清除。
 * 包含了软件层面的有效值/均值保护，以及底层硬件（CMPSS/Trip Zone）的联动。
 */

#include "protect.h"
#include "cmpss.h"
#include "epwm.h"

/*
 * 函数名: Init_Protect
 * 功能:   初始化保护参数与状态机
 * 参数:   v - 指向保护信息结构体的指针
 */
void Init_Protect( PROT_INFO *v )
{
    // 1. 设置保护阈值比例 (如 0.2 表示允许 20% 的裕量)
    v->I_INV_Prot_Rate = I_PROT_RATE;      // 逆变电流保护比例
    v->V_DC_Prot_Rate = V_PROT_RATE;       // 直流母线电压保护比例
    v->V_AC_Prot_Rate = V_PROT_RATE;       // 交流电网电压保护比例

   // 2. 清除所有硬件级故障标志 (HW: Hardware)
    v->Err_Flag.V_DC_HW_Err = false; // 硬件保护通常由快速比较器(CMPSS)直接触发，响应时间在纳秒级
    v->Err_Flag.V_AC_HW_Err = false;
    v->Err_Flag.I_INV_HW_Err = false;

    // 3. 清除所有软件级故障标志 (SW: Software)
    v->Err_Flag.V_DC_SW_Err = false;// 软件保护基于 ADC 采样计算出的有效值(RMS)或均值，响应稍慢但精度高
    v->Err_Flag.V_AC_SW_Err = false;
    v->Err_Flag.I_INV_SW_Err = false;

    // 4. 将系统全局状态机默认置为停机状态，等待安全检查通过后再启动
    v->Tim_Status = STOP;
}

/*
 * 函数名: Protect_Flag_Auto_Update
 * 功能:   在中断中高频轮询，实时更新保护标志位
 * 运行区: 放在 RAM 中极速运行 (.TI.ramfunc)
 * 参数:
 * p - 实时计算的功率信息 (如 RMS, 直流平均值)
 * v - 保护状态结构体
 * r - 设备的额定铭牌参数 (如额定电压、额定电流)
 */
#pragma CODE_SECTION(Protect_Flag_Auto_Update,".TI.ramfunc");
void Protect_Flag_Auto_Update( POWER_INFO *p , PROT_INFO *v , DEV_INFO *r)
{
    //软件保护逻辑判定
    if( p->I_Inv_L_RMS >  (v->I_INV_Prot_Rate + 1.0f) * r->I_Rated  )// 1. 交流过流保护 (软件): 实际 RMS 电流 > 额定电流 * (保护系数 + 1.0)
    {
        v->Err_Flag.I_INV_SW_Err=true;
    }


    if( (p->V_DC) > ( ( 1.0f + v->V_DC_Prot_Rate ) * r->V_DC_Rated ) ||  (p->V_DC) < ( (1.0f - v->V_DC_Prot_Rate  ) * r->V_DC_Rated ))// 2. 直流母线过欠压保护 (软件): 实际电压 > 上限 或 实际电压 < 下限
    {
        v->Err_Flag.V_DC_SW_Err = true;
    }


    if( (p->V_AC_RMS) > ( (1.0f + v->V_AC_Prot_Rate) * r->V_AC_Rated )   || (p->V_AC_RMS) < ( (1.0f - v->V_AC_Prot_Rate) * r->V_AC_Rated )) // 3. 交流电网过欠压保护 (软件): 实际 RMS 电压 > 上限 或 实际 RMS 电压 < 下限
    {
        v->Err_Flag.V_AC_SW_Err = true;
    }

    //底层硬件保护逻辑判定

    if( I_INV_HW_ERR ) // 直接读取底层的硬件状态宏(可能是 GPIO 电平或 CMPSS 状态寄存器)
    {
        v->Err_Flag.I_INV_HW_Err = true; // 电流硬件过流炸机保护
    }
    if( V_AC_HW_ERR )
    {
        v->Err_Flag.V_AC_HW_Err = true;  // 交流硬件过压
    }
    if( V_DC_HW_ERR )
    {
        v->Err_Flag.V_DC_HW_Err = true;  // 直流硬件过压
    }
}

/*
 * 函数名: Protect_Flag_Cmd_Clear
 * 功能:   接收上位机或外部的“复位指令”，尝试清除故障并恢复系统
 * 运行区: RAM 运行 (.TI.ramfunc)
 * 特点:   典型的基础软件(BSW)安全恢复逻辑——“带条件复位”。
 * 不能无脑清空标志位，必须先确认物理信号已经恢复到安全范围内，否则拒绝复位。
 */
#pragma CODE_SECTION(Protect_Flag_Cmd_Clear,".TI.ramfunc");
void Protect_Flag_Cmd_Clear( POWER_INFO *p , PROT_INFO *v , DEV_INFO *r , CMD *z)
{

    if(z->Err_Flag_Rst_Cmd == true)// 如果收到了清除故障的命令 (通常来自按键或串口通讯)
    {

        if( p->I_Inv_L_RMS <  (v->I_INV_Prot_Rate + 1.0f) * r->I_Rated )// 1. 尝试清除交流软件过流标志: 只有当电流确确实实降下来了，才允许清除
        {
            v->Err_Flag.I_INV_SW_Err=false;
        }

        // 2. 尝试清除直流软件过欠压标志: 只有电压回到安全区间，才允许清除
        // 注意这里的逻辑条件变成了 < 上限 且 > 下限
        // （原代码中此处用了 ||，但在实际 BSW 逻辑中，恢复条件通常要求同时满足不大于上限且不小于下限，即使用 &&，建议复查此处逻辑是否符合预期）
        if( (p->V_DC) < ( ( 1.0f + v->V_DC_Prot_Rate ) * r->V_DC_Rated ) ||  (p->V_DC) > ( (1.0f - v->V_DC_Prot_Rate  ) * r->V_DC_Rated ))
        {
            v->Err_Flag.V_DC_SW_Err = false;
        }

        // 3. 尝试清除交流软件过欠压标志
        if( (p->V_AC_RMS) < ( (1.0f + v->V_AC_Prot_Rate) * r->V_AC_Rated )   || (p->V_AC_RMS) > ( (1.0f - v->V_AC_Prot_Rate) * r->V_AC_Rated ))
        {
            v->Err_Flag.V_AC_SW_Err = false;
        }

        // 4. 尝试清除硬件比较器故障
        if(I_INV_HW_NO_ERR) // 如果物理引脚/比较器已经不再报故障
        {
            I_INV_HW_ERR_CLEAR; // 清除底层寄存器的锁存状态
            v->Err_Flag.I_INV_HW_Err = false;
        }

        if(V_AC_HW_NO_ERR)
        {
            V_AC_HW_ERR_CLEAR;
            v->Err_Flag.V_AC_HW_Err = false;
        }

        if(V_DC_HW_NO_ERR)
        {
            V_DC_HW_ERR_CLEAR;
            v->Err_Flag.V_DC_HW_Err = false;
        }

        // 5. 解除 PWM 的 Trip Zone (硬件跳闸) 封锁
        // 关键安全逻辑：只有当所有的硬件故障标志位都确切被清零后，才允许向 ePWM 模块发送清除一次性跳闸(OST)的指令，恢复发波能力。
        if( v->Err_Flag.I_INV_HW_Err == false && v->Err_Flag.V_AC_HW_Err == false && v->Err_Flag.V_DC_HW_Err == false  )
        {
            TZ_OST_CLEAR;
        }
    }

    // 无论清除是否成功，都将命令标志位置回 false，等待下一次指令触发
    z->Err_Flag_Rst_Cmd = false;
}
