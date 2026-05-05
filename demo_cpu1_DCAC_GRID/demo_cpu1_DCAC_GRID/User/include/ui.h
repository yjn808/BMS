/*
 * ui.h
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */

#ifndef USER_INCLUDE_UI_H_
#define USER_INCLUDE_UI_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

typedef struct {
    // 直流电流采样系数 (DC Current)
    float I_DC_Gain;        // 直流电流增益系数 [A/LSB],将ADC码值转换为实际电流值
    float I_DC_Offset;      // 直流电流偏置系数 [A],补偿传感器零点偏差

    // 逆变器桥臂侧电流采样系数 (Inverter Bridge-Side Current)
    float I_Inv_L_Gain;     // 桥臂侧电流增益系数 [A/LSB]
    float I_Inv_L_Offset;   // 桥臂侧电流偏置系数 [A]

    // 逆变器网侧电流采样系数 (Inverter Grid-Side Current)
    float I_Inv_G_Gain;     // 网侧电流增益系数 [A/LSB],用于电能质量监测
    float I_Inv_G_Offset;   // 网侧电流偏置系数 [A]

    // 交流电网电压采样系数 (AC Grid Voltage)
    float V_AC_Gain;        // 交流电压增益系数 [V/LSB]
    float V_AC_Offset;      // 交流电压偏置系数 [V]

    // 直流母线电压采样系数 (DC Bus Voltage)
    float V_DC_Gain;        // 直流电压增益系数 [V/LSB]
    float V_DC_Offset;      // 直流电压偏置系数 [V]

} SAMP_COEFF;


typedef struct {

    // 额定参数 (Rated Parameters)
    float V_AC_Rated;       // 额定交流电压 [V RMS],例如220V或110V,用于保护阈值计算
    float V_FRE_Rated;      // 额定电网频率 [Hz],例如50Hz或60Hz,用于PLL初始化
    float V_DC_Rated;       // 额定直流电压 [V],例如48V,用于保护阈值和占空比计算
    float I_Rated;          // 额定输出电流 [A RMS],例如10A,用于过流保护阈值
    float LINV_R;           // 逆变器滤波电感等效电阻 [Ω],用于控制器参数计算(预留)

    // 设备标识信息 (Device Identification)
    char Dev_Info[40];      // 设备信息字符串,例如"DC/AC Inverter 1kW"
    char Code_Info[20];     // 软件版本号,例如"25_11_21_V1.0",用于版本管理

} DEV_INFO;


typedef struct {

    // === 基本电气量 (Basic Electrical Quantities) ===
    float V_DC;             // 直流母线电压瞬时值 [V],经过10Hz低通滤波
    float I_DC;             // 直流输入电流瞬时值 [A],经过10Hz低通滤波
    float P_DC;             // 直流输入功率 [W],计算公式: P_DC = V_DC * I_DC

    // === RMS有效值 (RMS Values) ===
    float V_AC_RMS;         // 交流电网电压有效值 [V RMS],每20ms更新(800个采样点)
    float I_Inv_L_RMS;      // 逆变器桥臂侧电流有效值 [A RMS],用于控制和保护
    float I_Inv_G_RMS;      // 逆变器网侧电流有效值 [A RMS],用于电能质量监测

    // === 谐波分析 (Harmonic Analysis) ===
    float V_AC_Harmonic[10];      // 交流电压谐波分量数组 [V]
                                   // [0]: DC偏置
                                   // [1]: 基波(1次谐波,实际为0,PLL已滤除)
                                   // [2~9]: 2~9次谐波幅值(预留,当前未实现)

    float I_Inv_G_Harmonic[10];   // 网侧电流谐波分量数组 [A]
                                   // [0]: DC偏置
                                   // [1~9]: 各次谐波幅值(预留,当前未实现)

    // === 总谐波畸变率 (Total Harmonic Distortion) ===
    float V_AC_THD;               // 电压THD [%],计算公式: THD = √(H₂²+H₃²+...+H₉²) / H₁ × 100%
    float I_Inv_G_THD;            // 电流THD [%],评估电能质量指标

    // === 基波有功/无功分量 (Fundamental Active/Reactive Components) ===
    float I_Inv_G_Fundamental_Active;     // 网侧电流基波有功分量 [A],同相分量
    float I_Inv_G_Fundamental_Reactive;   // 网侧电流基波无功分量 [A],正交分量

    // === 基波功率计算 (Fundamental Power) ===
    float Power_Fundamental_Active;       // 基波有功功率 [W],实际输出功率
    float Power_Fundamental_Reactive;     // 基波无功功率 [Var],电网交换功率

    // === 电能质量指标 (Power Quality Indices) ===
    float S_Fundamental;          // 基波视在功率 [VA],计算公式: S = √(P² + Q²)
    float PF_Fundamental;         // 基波功率因数 [-],计算公式: PF = P / S,取值范围[0,1]
    float FRQ_Fundamental;        // 基波频率 [Hz],由PLL测量得到,用于频率监测

} POWER_INFO;



enum TIM_STAT
{
    STOP = 0x0000,          // 停机状态,PWM禁止输出
    RUNNING                 // 运行状态,PWM正常输出
};

typedef struct {

    bool V_DC_HW_Err;       // 直流电压硬件故障
                                // 触发条件: CMPSS5比较器检测到V_DC超出[V_DC_MIN, V_DC_MAX]
                                // 响应动作: TripZone立即关闭PWM(硬件联锁)

        bool V_AC_HW_Err;       // 交流电压硬件故障
                                // 触发条件: CMPSS2比较器检测到V_AC超出2048±1310
                                // 响应动作: TripZone立即关闭PWM

        bool I_INV_HW_Err;      // 逆变器电流硬件故障
                                // 触发条件: CMPSS3比较器检测到I_Inv_L超出2048±1638(约4.2A)
                                // 响应动作: TripZone立即关闭PWM

        // === 软件故障标志 (Software Fault Flags) ===
        bool V_DC_SW_Err;       // 直流电压软件故障
                                // 触发条件: |V_DC - V_DC_Rated| > 20% * V_DC_Rated
                                // 响应动作: 下一个控制周期停机(软件联锁)

        bool V_AC_SW_Err;       // 交流电压软件故障
                                // 触发条件: |V_AC_RMS - V_AC_Rated| > 20% * V_AC_Rated
                                // 响应动作: 下一个控制周期停机

        bool I_INV_SW_Err;      // 逆变器电流软件故障
                                // 触发条件: I_Inv_L_RMS > 120% * I_Rated
                                // 响应动作: 下一个控制周期停机

} ERR_FLAG;


typedef struct {

    float V_DC_Prot_Rate;   // 直流电压保护容差系数,默认0.2 (即±20%)
                            // 实际阈值: [V_DC_Rated*(1-0.2), V_DC_Rated*(1+0.2)]

    float V_AC_Prot_Rate;   // 交流电压保护容差系数,默认0.2 (即±20%)
                            // 实际阈值: [V_AC_Rated*(1-0.2), V_AC_Rated*(1+0.2)]

    float I_INV_Prot_Rate;  // 逆变器电流保护容差系数,默认0.2 (即120%)
                            // 实际阈值: I_Rated * (1 + 0.2) = 1.2倍额定电流

    // === 故障标志集合 (Fault Flags) ===
    ERR_FLAG Err_Flag;      // 包含6个故障标志的结构体,详见ERR_FLAG定义

    // === 运行状态 (Timing Status) ===
    enum TIM_STAT Tim_Status;  // 系统运行状态(STOP/RUNNING)
                                // 作用: 控制PWM输出的总开关
                                // 读取位置: PWM_Modulation_Ctrl()根据此标志使能/禁止PWM

} PROT_INFO;


enum CTRL_LOOP
{
    CLOSE_IN_LOOP = 0x0000,
    CLOSE_IN_OUT_LOOP
};


typedef struct {


    enum CTRL_LOOP Ctrl_Loop;  // only stop can write

    bool Feed_Ford_Enable;
    bool Dead_Time_Comp_Enable;

    float Out_Loop_Kp;
    float Out_Loop_Ki;

    float In_Loop_Kp;

    float P_In_Loop_Ref;// only in loop can write
    float Q_In_Loop_Ref;// only in loop can write

    float P_Out_Loop_Ref;// only close loop can write
    float Q_Out_Loop_Ref;// only close loop can write

} CTRL;


enum CTRL_CMD
{
    NO_CMD = 0x0000,
    START_CMD,
    STOP_CMD
};

typedef struct {
    enum CTRL_CMD Ctrl_Cmd;
    bool Err_Flag_Rst_Cmd;

} CMD;



//user only read
extern SAMP_COEFF Samp_Coff;//parameter
extern DEV_INFO Dev_Info;//parameter

extern POWER_INFO Power_Info;//power calc

extern PROT_INFO Prot_Info;//protect

//user read & write
extern CTRL Ctrl;//ctrl

//user only set
extern CMD Cmd;//timing


#endif /* USER_INCLUDE_UI_H_ */
