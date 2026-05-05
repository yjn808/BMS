/*
 * cmpss.h
 *
 * 创建年份: 2025
 * 作者: MAO
 * 描述: 比较器子系统 (CMPSS) 模块 2、3 和 5 配置的头文件。
 * 包含寄存器位定义、硬件故障状态标志位、保护阈值参数以及模块初始化
 * 的函数原型。本头文件支持基于硬件的电流 (CMPSS3)、交流电压 (CMPSS2)
 * 和直流电压 (CMPSS5) 监测，并集成了故障检测与保护机制。
 * 所有配置均采用 DAC 直接赋值模式（不使用影子寄存器），并禁用了斜坡发生器功能。
 */

#ifndef DSP_INCLUDE_CMPSS_H_
#define DSP_INCLUDE_CMPSS_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

// CMPSS 信号的输出 XBAR (交叉开关) 多路复用分配
#define  OUTPUTXBAR_4__CMPSS_2_H   2   // 将 CMPSS2 高边比较器输出映射到 OUTPUTXBAR4
#define  OUTPUTXBAR_8__CMPSS_2_L   3   // 将 CMPSS2 低边比较器输出映射到 OUTPUTXBAR8
#define  OUTPUTXBAR_2__CMPSS_3_H   4   // 将 CMPSS3 高边比较器输出映射到 OUTPUTXBAR2
#define  OUTPUTXBAR_3__CMPSS_3_L   5   // 将 CMPSS3 低边比较器输出映射到 OUTPUTXBAR3

// 硬件故障状态标志位
// 电流监测故障标志位 (CMPSS3)：当高边或低边比较器锁存器被激活时置位(触发保护)
#define  I_INV_HW_ERR  ( Cmpss3Regs.COMPSTS.bit.COMPHLATCH || Cmpss3Regs.COMPSTS.bit.COMPLLATCH )

// 交流电压监测故障标志位 (CMPSS2)：当高边或低边比较器锁存器被激活时置位
#define  V_AC_HW_ERR   ( Cmpss2Regs.COMPSTS.bit.COMPHLATCH || Cmpss2Regs.COMPSTS.bit.COMPLLATCH )

// 直流电压监测故障标志位 (CMPSS5)：当高边或低边比较器锁存器被激活时置位
#define  V_DC_HW_ERR   ( Cmpss5Regs.COMPSTS.bit.COMPHLATCH || Cmpss5Regs.COMPSTS.bit.COMPLLATCH )

// 正常状态宏：当高/低边比较器均未触发时为 TRUE
// 电流监测正常状态
#define  I_INV_HW_NO_ERR  ( Cmpss3Regs.COMPSTS.bit.COMPHSTS == 0 && Cmpss3Regs.COMPSTS.bit.COMPLSTS == 0 )

// 交流电压监测正常状态
#define  V_AC_HW_NO_ERR   ( Cmpss2Regs.COMPSTS.bit.COMPHSTS == 0 && Cmpss2Regs.COMPSTS.bit.COMPLSTS == 0 )

// 直流电压监测正常状态
#define  V_DC_HW_NO_ERR   ( Cmpss5Regs.COMPSTS.bit.COMPHSTS == 0 && Cmpss5Regs.COMPSTS.bit.COMPLSTS == 0 )

// 清除 CMPSS3 (电流) 故障锁存器的宏
// 必须在 EALLOW 受保护的环境下执行寄存器清零
#define  I_INV_HW_ERR_CLEAR                                                     \
        do                                                                      \
        {                                                                       \
        EALLOW;                                                                 \
        Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR = 1;                                \
        Cmpss3Regs.COMPSTSCLR.bit.LLATCHCLR = 1;                                \
        EDIS;                                                                   \
        }while(0)

// 清除 CMPSS2 (交流电压) 故障锁存器的宏
#define  V_AC_HW_ERR_CLEAR                                                      \
        do                                                                      \
        {                                                                       \
        EALLOW;                                                                 \
        Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1;                                \
        Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1;                                \
        EDIS;                                                                   \
        }while(0)

// 清除 CMPSS5 (直流电压) 故障锁存器的宏
#define  V_DC_HW_ERR_CLEAR                                                      \
        do                                                                      \
        {                                                                       \
        EALLOW;                                                                 \
        Cmpss5Regs.COMPSTSCLR.bit.HLATCHCLR = 1;                                \
        Cmpss5Regs.COMPSTSCLR.bit.LLATCHCLR = 1;                                \
        EDIS;                                                                   \
        }while(0)

// 保护阈值参数设定 (DAC 给定值)
// 最大允许电流阈值 (CMPSS3)。基准值 1310 约对应 4.2A，这里做了 25% 的抗扰动扩展裕量
#define I_INVL_MAX      (1310 + (1310 >> 2))
// 最大允许交流电压阈值 (CMPSS2)。数值 1310 约对应 55V
#define V_AC_MAX        (1310)
// 最大允许直流电压阈值 (CMPSS5)。数值 4050 约对应 55V
#define V_DC_MAX        (4050)  //~55V
// 最小允许直流电压阈值 (CMPSS5)。数值 2960 约对应 40V
#define V_DC_MIN        (2960)  //~40V

// CMPSS 模块功能使能控制
#define CMPSS_ENABLE      1   // 使能 CMPSS 模块
#define CMPSS_DISENABLE   0   // 禁用 CMPSS 模块

// DAC 参考电压源选择
#define REFERENCE_VDDA     0   // 选择外部 VDDA 作为 DAC 参考电压
#define REFERENCE_VDAC     1   // 选择内部 VDAC 作为 DAC 参考电压

// 比较器负端输入源选择
#define NEGIN_DAC          0   // 使用内部 DAC 输出作为比较器负端输入
#define NEGIN_PIN          1   // 使用外部引脚输入作为比较器负端输入

// 跳闸 (Trip) 信号输出模式选择
#define CTRIP_ASYNCH       0   // 异步输出 (不与系统时钟同步，响应最快)
#define CTRIP_SYNCH        1   // 同步输出 (与系统时钟同步)
#define CTRIP_FILTER       2   // 滤波输出 (经过内部数字滤波器处理后输出)
#define CTRIP_LATCH        3   // 锁存输出 (触发后保持状态，直到通过代码显式清除)

// 函数原型声明
// 初始化用于电流监测的 CMPSS3 模块 (不使用斜坡发生器和 DAC 影子寄存器)
void Init_Cmpss_3();
// 初始化用于交流电压监测的 CMPSS2 模块 (不使用斜坡发生器和 DAC 影子寄存器)
void Init_Cmpss_2();
// 初始化用于直流电压监测的 CMPSS5 模块 (不使用斜坡发生器和 DAC 影子寄存器)
void Init_Cmpss_5();

#endif /* DSP_INCLUDE_CMPSS_H_ */
