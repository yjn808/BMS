#include "App_Display.h"  // 包含App_Display头文件，用于显示相关功能

extern uint32_t bat_mv;    // 外部变量：电池电压，单位毫伏
extern float current_a;    // 外部变量：电流，单位安培

extern int8_t tempr;       // 外部变量：温度，单位摄氏度

extern uint8_t soc_percent; // 外部变量：电量百分比，单位百分比

/**
 * @brief 初始化显示模块
 * @details 初始化OLED显示屏，为后续显示操作做准备
 */
void App_Display_Init(void)
{
    Inf_OLED_Init();  // 调用底层OLED初始化函数
}

/**
 * @brief 显示标题信息
 * @details 在OLED屏幕上显示"BMS"系统标题
 */
static void show_title(void)
{ // 
    for (size_t i = 0; i < 5; i++)
    {
        if (i <= 2)
        {
            Inf_OLED_ShowChinese(8 + i * 16, 8, i, 16, 1);
        }
        else
        {
            Inf_OLED_ShowChinese(8 + 24 + i * 16, 8, i, 16, 1);
        }
    }
    Inf_OLED_ShowString(8 + 16 * 3, 8, (uint8_t *)"BMS", 16, 1);
}

static void show_battery_data(void)
{
    // 电压
    char v_str[10] = {0};
    sprintf(v_str, "V:%5.2f", bat_mv / 1000.0);
    Inf_OLED_ShowString(8, 8 + 16, (uint8_t *)v_str, 16, 1);
    // 电流
    char a_str[10] = {0};
    sprintf(a_str, "A:%5.2f", current_a);
    Inf_OLED_ShowString(70, 8 + 16, (uint8_t *)a_str, 16, 1);

    // 温度
    char t_str[10] = {0};
    sprintf(t_str, "T:%d", tempr);
    Inf_OLED_ShowString(8, 8 + 16 * 2, (uint8_t *)t_str, 16, 1);
    // 电量
    char s_str[10] = {0};
    sprintf(s_str, "S:%d%%", soc_percent);
    Inf_OLED_ShowString(70, 8 + 16 * 2, (uint8_t *)s_str, 16, 1);
}

/**
 * @brief 显示
 */
void App_Display_Show(void)
{
    show_title();

    show_battery_data();
    taskENTER_CRITICAL();
    Inf_OLED_Refresh();
    taskEXIT_CRITICAL();
}
