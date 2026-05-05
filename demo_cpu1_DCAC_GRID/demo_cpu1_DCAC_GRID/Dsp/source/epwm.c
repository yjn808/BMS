/*
 * epwm.c
 *
 *  Created on: 2025
 *  实现TI C2000 F2837xD 系列中 EPWM1 和 EPWM4 的底层配置。
 *
 *  主要功能包括：
 *  1. 指定 EPWM1 / EPWM4 由 CPU1 控制
 *  2. 设置 ePWM 模块时钟
 *  3. 配置时基计数器（上下载波）
 *  4. 配置比较器与影子寄存器
 *  5. 配置动作限定器 AQ，生成 PWM A 基础波形
 *  6. 配置 Dead-Band，自动生成互补上下管驱动
 *  7. 配置 EPWM1 触发 ADC 采样
 *  8. 配置 CMPSS -> TRIP4 -> ePWM Trip Zone 保护链
 *  9. 提供 PWM 输出使能 / 关断函数
 */

#include "epwm.h"

/**
 * @brief 选择 CPU1 作为 EPWM1 和 EPWM4 的控制核心
 *
 * 说明：
 * F2837xD 是双核芯片，很多外设可以分配给 CPU1 或 CPU2。
 * 这里把 EPWM1 和 EPWM4 都分给 CPU1 控制。
 */
void Epwm_Cpu1_Sel(void)
{
    EALLOW;   // 允许写受保护寄存器

    DevCfgRegs.CPUSEL0.bit.EPWM1 = 0;   // EPWM1 归 CPU1 控制
    DevCfgRegs.CPUSEL0.bit.EPWM4 = 0;   // EPWM4 归 CPU1 控制

    EDIS;     // 关闭受保护寄存器写权限
}

/**
 * @brief 设置 ePWM 模块的外设时钟分频
 *
 * 说明：
 * 这里把 ePWM 时钟分频设置为 1，也就是不再额外分频。
 * 这样 ePWM 工作时钟就是 100MHz（由工程中的时钟设置决定）。
 */
void Epwm_Clk_Div_Sel(void)
{
    EALLOW;

    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;   // ePWM 时钟不分频

    EDIS;
}

/**
 * @brief 同步启动所有 ePWM 模块的时基计数器
 *
 * 说明：
 * TBCLKSYNC=1 后，所有 ePWM 的 TBCTR 开始统一计数。
 * 这样多路 PWM 能同步运行。
 */
void Epwm_Count_Start(void)
{
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // 打开所有 ePWM 的时基时钟同步

    EDIS;
}

/**
 * @brief 停止所有 ePWM 模块的时基计数器
 *
 * 说明：
 * 在配置 PWM 之前，先把计数器停掉，避免配置过程中输出错误脉冲。
 */
void Epwm_Count_Stop(void)
{
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // 关闭时基同步，停止计数

    EDIS;
}

/**
 * @brief 初始化 EPWM1 和 EPWM4
 *
 * 作用：
 * 这是整个文件最核心的函数。
 * 它把 EPWM1、EPWM4 配成一组可用于单相全桥逆变器的 PWM 输出：
 *
 * - EPWM1：主模块
 * - EPWM4：从模块
 * - 二者同步
 * - 上下载波
 * - 中心对称 PWM
 * - 带死区
 * - EPWM1 触发 ADC 采样
 * - CMPSS 故障信号接入 Trip Zone 实现硬件关断
 */
void Init_Epwm1_Epwm4(void)
{
    // 第一步：配置前先停掉 PWM 计数器，避免乱输出
    Epwm_Count_Stop();

    // 第二步：设置同步链
    // EPWM1 做主同步源，EPWM4 跟着 EPWM1 同步
    EALLOW;
    SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0x0;    // EPWM4 的同步输入来自 EPWM1SYNCOUT
    SyncSocRegs.SYNCSELECT.bit.EPWM7SYNCIN = 0x0;    // 预留
    SyncSocRegs.SYNCSELECT.bit.EPWM10SYNCIN = 0x0;   // 预留
    EDIS;

    /******************** 配置 EPWM1（主桥臂） ********************/

    // 1) 时基配置
    EPwm1Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UPDOWN;   // 上下载波模式，生成中心对称 PWM
    EPwm1Regs.TBPRD                = TB_TBPRD;          // 设置 PWM 周期
    EPwm1Regs.TBCTL.bit.PHSEN      = TB_DISABLE;        // 主模块，不使用外部同步相位装载
    EPwm1Regs.TBPHS.bit.TBPHS      = 0x0000;            // 相位初值 = 0
    EPwm1Regs.TBCTR                = 0x0000;            // 计数器清零
    EPwm1Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV1;           // 高速时钟不分频
    EPwm1Regs.TBCTL.bit.CLKDIV     = TB_DIV1;           // 时基时钟不分频

    // 2) 同步输出配置
    EPwm1Regs.TBCTL.bit.SYNCOSEL   = TB_CTR_ZERO;       // TBCTR=0 时输出同步信号，给从模块用

    // 3) 比较器配置
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // CMPA 使用影子寄存器，避免更新毛刺
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   // 在计数到 0 或 PRD 时装载新 CMPA
    EPwm1Regs.CMPA.bit.CMPA        = TB_TBPRD >> 1;     // 初始 50% 占空比

    // 4) 动作限定器 AQ 配置
    // CAU：计数向上数时，遇到 CMPA
    // CAD：计数向下数时，遇到 CMPA
    EPwm1Regs.AQCTLA.bit.CAD       = AQ_SET;            // 下数碰到 CMPA，EPWM1A 置位
    EPwm1Regs.AQCTLA.bit.CAU       = AQ_CLEAR;          // 上数碰到 CMPA，EPWM1A 清零
    // 这样可以得到中心对称 PWM 波形

    // 5) 死区模块配置
    EPwm1Regs.DBCTL.bit.OUT_MODE   = DB_FULL_ENABLE;    // 开启全死区，自动生成 A/B 互补
    EPwm1Regs.DBCTL.bit.POLSEL     = DB_ACTV_HIC;       // 高电平有效互补输出
    EPwm1Regs.DBRED.bit.DBRED      = DETIM_1US;         // 上升沿死区 = 1us
    EPwm1Regs.DBFED.bit.DBFED      = DETIM_1US;         // 下降沿死区 = 1us

    // 6) 配置 EPWM1 触发 ADC
    EPwm1Regs.ETSEL.bit.SOCAEN     = ET_ENABLE;         // 允许 SOCA
    EPwm1Regs.ETSEL.bit.SOCASEL    = ET_CTR_PRDZERO;    // 在 TBCTR = PRD 或 ZERO 时触发 ADC
    EPwm1Regs.ETPS.bit.SOCAPRD     = ET_1ST;            // 每次事件都触发一次

    /******************** 配置 EPWM4（从桥臂） ********************/

    // 1) 时基配置
    EPwm4Regs.TBCTL.bit.CTRMODE    = TB_COUNT_UPDOWN;   // 与 EPWM1 一样，上下载波
    EPwm4Regs.TBPRD                = TB_TBPRD;          // 周期与主模块一致
    EPwm4Regs.TBCTL.bit.HSPCLKDIV  = TB_DIV1;           // 不分频
    EPwm4Regs.TBCTL.bit.CLKDIV     = TB_DIV1;           // 不分频
    EPwm4Regs.TBCTR                = 0x0000;            // 清零计数器

    // 2) 从模块同步设置
    EPwm4Regs.TBCTL.bit.PHSEN      = TB_ENABLE;         // 允许相位装载
    EPwm4Regs.TBCTL.bit.PHSDIR     = TB_UP;             // 接收到同步后向上计数
    EPwm4Regs.TBPHS.bit.TBPHS      = 0x0000;            // 初始相位 = 0
    EPwm4Regs.TBCTL.bit.SYNCOSEL   = TB_SYNC_DISABLE;   // 从模块不再往外发同步

    // 3) 比较器配置
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // 影子寄存器
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   // 0 或 PRD 装载新值
    EPwm4Regs.CMPA.bit.CMPA        = TB_TBPRD >> 1;     // 初始 50% 占空比

    // 4) 动作限定器 AQ 配置
    EPwm4Regs.AQCTLA.bit.CAD       = AQ_SET;            // 下数碰到 CMPA，EPWM4A 置位
    EPwm4Regs.AQCTLA.bit.CAU       = AQ_CLEAR;          // 上数碰到 CMPA，EPWM4A 清零

    // 5) 死区配置
    EPwm4Regs.DBCTL.bit.OUT_MODE   = DB_FULL_ENABLE;    // 开启全死区
    EPwm4Regs.DBCTL.bit.POLSEL     = DB_ACTV_HIC;       // 互补输出
    EPwm4Regs.DBRED.bit.DBRED      = DETIM_1US;         // 上升沿死区 1us
    EPwm4Regs.DBFED.bit.DBFED      = DETIM_1US;         // 下降沿死区 1us

    /******************** 配置故障保护链路 ********************/

    EALLOW;

    // 这里把 CMPSS2/3/5 的高低比较器输出都映射到 TRIP4
    // 也就是说：
    // - 电流保护
    // - 交流电压保护
    // - 直流电压保护
    // 最终都汇总到 ePWM 的 TRIP4 保护输入上

    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4 = 0;   // CMPSS3H -> TRIP4
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX5 = 0;   // CMPSS3L -> TRIP4
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2 = 0;   // CMPSS2H -> TRIP4
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 0;   // CMPSS2L -> TRIP4
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX8 = 0;   // CMPSS5H -> TRIP4
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX9 = 0;   // CMPSS5L -> TRIP4

    // 使能以上 mux
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX5 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX8 = 1;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX9 = 1;

    // 不反相 TRIP4
    EPwmXbarRegs.TRIPOUTINV.bit.TRIP4 = 0;

    /******************** 配置 EPWM1 的 Trip Zone ********************/

    EPwm1Regs.TZCLR.bit.OST = 1;                       // 清除一次性 Trip 标志
    EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_TRIPIN4; // DCAH 输入 = TRIP4
    EPwm1Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;       // DCAH 高电平触发 DCAEVT1
    EPwm1Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT1;        // 事件源 = DCAEVT1
    EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC; // 异步故障，尽快响应
    EPwm1Regs.TZSEL.bit.DCAEVT1 = TZ_ENABLE;          // 允许 DCAEVT1 成为关断源

    /******************** 配置 EPWM4 的 Trip Zone ********************/

    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_TRIPIN4;
    EPwm4Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;
    EPwm4Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT1;
    EPwm4Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
    EPwm4Regs.TZSEL.bit.DCAEVT1 = TZ_ENABLE;

    /******************** Trip 后输出动作：全关断 ********************/

    EPwm1Regs.TZCLR.bit.CBCPULSE = 3;                 // 设置 CBC 清除脉冲行为
    EPwm1Regs.TZCTL.bit.TZA   = TZ_FORCE_LO;          // 发生故障时 EPWM1A 拉低
    EPwm1Regs.TZCTL.bit.TZB   = TZ_FORCE_LO;          // 发生故障时 EPWM1B 拉低
    EPwm1Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;        // 故障事件时强制低电平
    EPwm1Regs.TZCTL.bit.DCAEVT2 = TZ_FORCE_LO;
    EPwm1Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO;
    EPwm1Regs.TZCTL.bit.DCBEVT2 = TZ_FORCE_LO;
    EPwm1Regs.TZFRC.bit.CBC = 1;                      // 上电默认先保持关断

    EPwm4Regs.TZCLR.bit.CBCPULSE = 3;
    EPwm4Regs.TZCTL.bit.TZA   = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.TZB   = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.DCAEVT2 = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.DCBEVT1 = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.DCBEVT2 = TZ_FORCE_LO;
    EPwm4Regs.TZFRC.bit.CBC = 1;                      // 上电默认先保持关断

    EDIS;
}

/**
 * @brief 使能 PWM 输出
 *
 * 说明：
 * 这个函数的作用不是“重新配置 PWM”，
 * 而是把前面由于 CBC 关断锁住的输出释放出来，
 * 让 EPWM1 / EPWM4 恢复正常输出。
 */
#pragma CODE_SECTION(epwm_output_enable,".TI.ramfunc");
void epwm_output_enable(void)
{
    EALLOW;

    EPwm1Regs.TZCLR.bit.CBCPULSE = 2;   // 清 CBC 锁存，允许 EPWM1 恢复输出
    EPwm4Regs.TZCLR.bit.CBCPULSE = 2;   // 清 CBC 锁存，允许 EPWM4 恢复输出

    EDIS;
}

/**
 * @brief 关闭 PWM 输出
 *
 * 说明：
 * 通过强制触发 CBC 故障事件，让 EPWM1 / EPWM4 的所有输出立即被拉低。
 * 这通常用于：
 * 1. 软件停机
 * 2. 保护动作
 * 3. 系统未进入运行状态时禁止发波
 */
#pragma CODE_SECTION(epwm_output_disable,".TI.ramfunc");
void epwm_output_disable(void)
{
    EALLOW;

    EPwm1Regs.TZCLR.bit.CBCPULSE = 3;   // 设置为 CBC 锁存状态
    EPwm4Regs.TZCLR.bit.CBCPULSE = 3;

    EPwm1Regs.TZFRC.bit.CBC = 1;        // 强制 EPWM1 触发 CBC 故障
    EPwm4Regs.TZFRC.bit.CBC = 1;        // 强制 EPWM4 触发 CBC 故障

    EDIS;
}

/**
 * @brief EPWM1 中断服务函数
 *
 * 说明：
 * 目前这个中断函数基本是空的，
 * 只做了中断标志清除和 PIE 应答。
 * 说明当前工程里主要控制任务并不是放在 ePWM 中断里跑，
 * 而是放在 ADC 中断里跑。
 */
#pragma CODE_SECTION(epwm1_isr,".TI.ramfunc");
interrupt void epwm1_isr(void)
{
#ifdef CPU1
    // CPU1 下可扩展的中断处理逻辑
#endif

#ifdef CPU2
    // CPU2 下可扩展的中断处理逻辑
#endif

    EPwm1Regs.ETCLR.bit.INT = 1;          // 清除 EPWM1 中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // 应答 PIE 第3组中断
}
