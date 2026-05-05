#ifndef __GNSS_H
#define __GNSS_H

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>

// GNSS数据结构体
typedef struct {
    // 基本信息
    char raw_data[256];         // 原始数据
    char data_type[10];         // 数据类型 GGA/RMC
    
    // 时间信息
    char time[15];              // UTC时间
    char friendly_time[20];     // 友好时间格式
    char date[15];              // 日期
    
    // 位置信息
    char latitude[15];          // 原始纬度
    char longitude[15];         // 原始经度
    double lat_degree;          // 纬度(度)
    double lon_degree;          // 经度(度)
    char status[20];            // 定位状态
    
    // GGA特有字段
    char quality[5];            // 定位质量
    char satellites[5];         // 卫星数量
    char hdop[10];              // 水平精度因子
    char altitude[15];          // 海拔高度
    char geoid_height[15];      // 大地水准面高度
    
    // RMC特有字段
    char position_status;       // 定位状态 A=有效,V=无效
    char speed[10];             // 地面速率(节)
    char course[10];            // 地面航向(度)
    char magnetic_variation[10]; // 磁偏角
    
    // 状态标志
    bool has_gga_data;
    bool has_rmc_data;
} GNSS_Data_t;

// 全局变量声明
extern char gnss_rx_buffer[512];       // GNSS接收缓冲区
extern uint16_t gnss_rx_index;         // 接收索引
extern GNSS_Data_t gnss_data;          // GNSS数据结构体
extern volatile bool gnss_data_ready;  // 数据就绪标志

// 函数声明

// 调试输出函数 (使用USART1)
void GNSS_SendString(char *str);
void GNSS_Printf(const char *format, ...);

// GNSS解析函数
void GNSS_InitData(GNSS_Data_t *data);
bool GNSS_ParseLine(char *line, GNSS_Data_t *data);
bool GNSS_ParseGGA(char *line, GNSS_Data_t *data);
bool GNSS_ParseRMC(char *line, GNSS_Data_t *data);
void GNSS_RepairData(char *data);

// 数据转换函数
void GNSS_ConvertTime(GNSS_Data_t *data);
void GNSS_ConvertToDegree(GNSS_Data_t *data);
void GNSS_ConvertDate(GNSS_Data_t *data);
void GNSS_ConvertSpeed(GNSS_Data_t *data);

// 显示函数
void GNSS_DisplayData(GNSS_Data_t *data);
void GNSS_DisplaySummary(GNSS_Data_t *data);

// 主处理函数
void GNSS_ProcessData(void);

// 初始化函数
void GNSS_Init(void);

#endif /* __GNSS_H */



