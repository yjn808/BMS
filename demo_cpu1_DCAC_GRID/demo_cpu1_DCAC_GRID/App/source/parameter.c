#include <string.h>
#include "parameter.h"


void Init_Samp_Coff(SAMP_COEFF *v)
{
    // 初始化直流侧电流采样系数
    v->I_DC_Gain = I_GAIN;        // 直流电流采样增益
    v->I_DC_Offset = I_OFFSET;    // 直流电流采样偏置

    // 初始化逆变器电感侧电流采样系数
    v->I_Inv_L_Gain = I_GAIN;     // 电感侧电流采样增益
    v->I_Inv_L_Offset = I_OFFSET; // 电感侧电流采样偏置

    // 初始化并网侧电流采样系数
    v->I_Inv_G_Gain = I_GAIN;     // 并网侧电流采样增益
    v->I_Inv_G_Offset = I_OFFSET; // 并网侧电流采样偏置

    // 初始化直流母线电压采样系数
    v->V_DC_Gain = V_DC_GAIN;     // 直流母线电压采样增益
    v->V_DC_Offset = V_DC_OFFSET; // 直流母线电压采样偏置

    // 初始化交流侧电压采样系数
    v->V_AC_Gain = V_AC_GAIN;     // 交流电压采样增益
    v->V_AC_Offset = V_AC_OFFSET; // 交流电压采样偏置
}




void Init_Dev_Info(DEV_INFO *v)
{
    v->V_AC_Rated = V_AC_RATED;     // 额定交流电压
    v->V_FRE_Rated = V_FRE_RATED;   // 额定交流频率
    v->V_DC_Rated = V_DC_RATED;     // 额定直流母线电压
    v->I_Rated = I_RATED;           // 额定电流
    v->LINV_R = L_R;                // 滤波电感等效参数/电感相关参数

    // 写入设备类型字符串
    strcpy(v->Dev_Info, " DC/AC");

    // 写入代码版本号/固件版本号
    strcpy(v->Code_Info, "25_11_21_V1.0");
}


