#include "gnss.h"


// 全局变量定义
char gnss_rx_buffer[512] = {0};        // GNSS接收缓冲区
uint16_t gnss_rx_index = 0;            // 接收索引
GNSS_Data_t gnss_data;                 // GNSS数据结构体
volatile bool gnss_data_ready = false; // 数据就绪标志

// USART1发送字符串函数
void GNSS_SendString(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// USART1格式化输出函数
void GNSS_Printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// 初始化GNSS数据
void GNSS_InitData(GNSS_Data_t *data) {
    memset(data, 0, sizeof(GNSS_Data_t));
    data->lat_degree = 0.0;
    data->lon_degree = 0.0;
    data->position_status = 'V'; // 默认无效
}

// 数据修复函数
void GNSS_RepairData(char *data) {
    // 移除开头不可见字符
    while (*data && (*data == '\n' || *data == '\r' || *data == ' ')) {
        memmove(data, data + 1, strlen(data));
    }
    
    // 修复常见数据头错误
    if (strlen(data) >= 5) {
        if (strncmp(data, "GMGGA", 5) == 0) data[1] = 'P';
        else if (strncmp(data, "GKGGA", 5) == 0) data[1] = 'N';
        else if (strncmp(data, "GMRMC", 5) == 0) data[1] = 'P';
        else if (strncmp(data, "KNRMC", 5) == 0) data[1] = 'N';
    }
    
    // 修复字符错误
    for (int i = 0; data[i]; i++) {
        switch (data[i]) {
            case 'I': data[i] = '1'; break;
            case 'O': data[i] = '0'; break;
            case 'l': data[i] = '1'; break;
        }
    }
    
    // 确保以$开头
    if (data[0] != '$' && (strstr(data, "GGA") || strstr(data, "RMC"))) {
        memmove(data + 1, data, strlen(data) + 1);
        data[0] = '$';
    }
    
    // 移除空格
    char *src = data, *dst = data;
    while (*src) {
        if (*src != ' ') *dst++ = *src;
        src++;
    }
    *dst = '\0';
}

// 解析数据
bool GNSS_ParseLine(char *line, GNSS_Data_t *data) {
    char line_copy[256];
    strncpy(line_copy, line, sizeof(line_copy)-1);
    line_copy[sizeof(line_copy)-1] = '\0';
    
    GNSS_RepairData(line_copy);
    strncpy(data->raw_data, line_copy, sizeof(data->raw_data)-1);
    
    if (strstr(line_copy, "GGA") != NULL) {
        strcpy(data->data_type, "GGA");
        return GNSS_ParseGGA(line_copy, data);
    } else if (strstr(line_copy, "RMC") != NULL) {
        strcpy(data->data_type, "RMC");
        return GNSS_ParseRMC(line_copy, data);
    }
    
    return false;
}

// 解析GGA数据
bool GNSS_ParseGGA(char *line, GNSS_Data_t *data) {
    int field = 0;
    char *token = strtok(line, ",");
    
    while (token != NULL && field < 15) {
        switch (field) {
            case 1: strncpy(data->time, token, sizeof(data->time)-1); break;
            case 2: strncpy(data->latitude, token, sizeof(data->latitude)-1); break;
            case 3: 
                if (strlen(data->status) == 0) {
                    strcpy(data->status, token[0]=='N' ? "北纬" : "南纬");
                }
                break;
            case 4: strncpy(data->longitude, token, sizeof(data->longitude)-1); break;
            case 5:
                if (strstr(data->status, "东经") == NULL && strstr(data->status, "西经") == NULL) {
                    strcat(data->status, token[0]=='E' ? " 东经" : " 西经");
                }
                break;
            case 6: strncpy(data->quality, token, sizeof(data->quality)-1); break;
            case 7: strncpy(data->satellites, token, sizeof(data->satellites)-1); break;
            case 8: strncpy(data->hdop, token, sizeof(data->hdop)-1); break;
            case 9: strncpy(data->altitude, token, sizeof(data->altitude)-1); break;
            case 11: strncpy(data->geoid_height, token, sizeof(data->geoid_height)-1); break;
        }
        field++;
        token = strtok(NULL, ",");
    }
    
    if (field >= 10) {
        data->has_gga_data = true;
        GNSS_ConvertTime(data);
        GNSS_ConvertToDegree(data);
        return true;
    }
    
    return false;
}

// 解析RMC数据
bool GNSS_ParseRMC(char *line, GNSS_Data_t *data) {
    int field = 0;
    char *token = strtok(line, ",");
    
    while (token != NULL && field < 13) {
        switch (field) {
            case 1: strncpy(data->time, token, sizeof(data->time)-1); break;
            case 2: 
                if (strlen(token) > 0) data->position_status = token[0];
                break;
            case 3: 
                if (strlen(data->latitude) == 0) {
                    strncpy(data->latitude, token, sizeof(data->latitude)-1);
                }
                break;
            case 4:
                if (strlen(data->status) == 0) {
                    strcpy(data->status, token[0]=='N' ? "北纬" : "南纬");
                }
                break;
            case 5:
                if (strlen(data->longitude) == 0) {
                    strncpy(data->longitude, token, sizeof(data->longitude)-1);
                }
                break;
            case 6:
                if (strstr(data->status, "东经") == NULL && strstr(data->status, "西经") == NULL) {
                    strcat(data->status, token[0]=='E' ? " 东经" : " 西经");
                }
                break;
            case 7: strncpy(data->speed, token, sizeof(data->speed)-1); break;
            case 8: strncpy(data->course, token, sizeof(data->course)-1); break;
            case 9: strncpy(data->date, token, sizeof(data->date)-1); break;
            case 10: strncpy(data->magnetic_variation, token, sizeof(data->magnetic_variation)-1); break;
        }
        field++;
        token = strtok(NULL, ",");
    }
    
    if (field >= 10) {
        data->has_rmc_data = true;
        GNSS_ConvertTime(data);
        GNSS_ConvertToDegree(data);
        GNSS_ConvertDate(data);
        GNSS_ConvertSpeed(data);
        return true;
    }
    
    return false;
}

// 时间转换
void GNSS_ConvertTime(GNSS_Data_t *data) {
    if (strlen(data->time) >= 6) {
        char hour[3] = {0}, minute[3] = {0}, second[3] = {0};
        strncpy(hour, data->time, 2);
        strncpy(minute, data->time + 2, 2);
        strncpy(second, data->time + 4, 2);
        
        int beijing_hour = (atoi(hour) + 8) % 24;
        snprintf(data->friendly_time, sizeof(data->friendly_time),
                "%02d:%02d:%02d", beijing_hour, atoi(minute), atoi(second));
    }
}

// 坐标转换
void GNSS_ConvertToDegree(GNSS_Data_t *data) {
    // 纬度转换
    if (strlen(data->latitude) >= 4) {
        char deg[3] = {0};
        strncpy(deg, data->latitude, 2);
        double degrees = atof(deg);
        double minutes = atof(data->latitude + 2);
        data->lat_degree = degrees + minutes / 60.0;
        if (strstr(data->status, "南纬")) data->lat_degree = -data->lat_degree;
    }
    
    // 经度转换
    if (strlen(data->longitude) >= 5) {
        char deg[4] = {0};
        strncpy(deg, data->longitude, 3);
        double degrees = atof(deg);
        double minutes = atof(data->longitude + 3);
        data->lon_degree = degrees + minutes / 60.0;
        if (strstr(data->status, "西经")) data->lon_degree = -data->lon_degree;
    }
}

// 日期转换
void GNSS_ConvertDate(GNSS_Data_t *data) {
    if (strlen(data->date) == 6) {
        char day[3] = {0}, month[3] = {0}, year[3] = {0};
        strncpy(day, data->date, 2);
        strncpy(month, data->date + 2, 2);
        strncpy(year, data->date + 4, 2);
        
        snprintf(data->date, sizeof(data->date), "20%s-%s-%s", year, month, day);
    }
}

// 速度转换 (节 -> 公里/小时)
void GNSS_ConvertSpeed(GNSS_Data_t *data) {
    if (strlen(data->speed) > 0) {
        double kmh = atof(data->speed) * 1.852;
        snprintf(data->speed, sizeof(data->speed), "%.2f", kmh);
    }
}

// 显示完整GNSS数据
void GNSS_DisplayData(GNSS_Data_t *data) {
    GNSS_SendString("\r\n");
    GNSS_SendString("=========================================\r\n");
    GNSS_SendString("           GNSS数据解析结果\r\n");
    GNSS_SendString("=========================================\r\n");
    
    // 基本信息
    GNSS_Printf("数据来源: ");
    if (data->has_gga_data) GNSS_SendString("GGA ");
    if (data->has_rmc_data) GNSS_SendString("RMC");
    GNSS_SendString("\r\n");
    
    GNSS_Printf("UTC时间: %s\r\n", data->time);
    GNSS_Printf("北京时间: %s\r\n", data->friendly_time);
    
    if (strlen(data->date) > 0) {
        GNSS_Printf("日期: %s\r\n", data->date);
    }
    
    // 位置信息
    GNSS_SendString("-----------------------------------------\r\n");
    GNSS_Printf("定位状态: %s\r\n", data->status);
    GNSS_Printf("原始坐标: %s, %s\r\n", data->latitude, data->longitude);
    GNSS_Printf("度格式坐标: %.6f, %.6f\r\n", data->lat_degree, data->lon_degree);
    
    // GGA特有信息
    if (data->has_gga_data) {
        GNSS_SendString("-----------------------------------------\r\n");
        GNSS_SendString("GGA定位信息:\r\n");
        GNSS_Printf("定位质量: %s", data->quality);
        if (strcmp(data->quality, "0") == 0) GNSS_SendString(" (无效)\r\n");
        else if (strcmp(data->quality, "1") == 0) GNSS_SendString(" (GPS定位)\r\n");
        else if (strcmp(data->quality, "2") == 0) GNSS_SendString(" (差分定位)\r\n");
        else GNSS_SendString("\r\n");
        
        GNSS_Printf("卫星数量: %s颗\r\n", data->satellites);
        GNSS_Printf("海拔高度: %s米\r\n", data->altitude);
        if (strlen(data->hdop) > 0) {
            GNSS_Printf("水平精度: %s\r\n", data->hdop);
        }
    }
    
    // RMC特有信息
    if (data->has_rmc_data) {
        GNSS_SendString("-----------------------------------------\r\n");
        GNSS_SendString("RMC导航信息:\r\n");
        GNSS_Printf("定位状态: %c", data->position_status);
        if (data->position_status == 'A') GNSS_SendString(" (有效)\r\n");
        else if (data->position_status == 'V') GNSS_SendString(" (无效)\r\n");
        else GNSS_SendString("\r\n");
        
        if (strlen(data->speed) > 0) {
            GNSS_Printf("地面速率: %s km/h\r\n", data->speed);
        }
        if (strlen(data->course) > 0) {
            GNSS_Printf("地面航向: %s°\r\n", data->course);
        }
        if (strlen(data->magnetic_variation) > 0) {
            GNSS_Printf("磁偏角: %s\r\n", data->magnetic_variation);
        }
    }
    
    GNSS_SendString("=========================================\r\n");
}

// 显示简要信息
void GNSS_DisplaySummary(GNSS_Data_t *data) {
    GNSS_Printf("?? 位置: %.6f, %.6f | ", data->lat_degree, data->lon_degree);
    GNSS_Printf("?? %s | ", data->friendly_time);
    
    if (data->has_gga_data) {
        GNSS_Printf("??? %s星 | ", data->satellites);
        GNSS_Printf("?? %sm", data->altitude);
    }
    
    if (data->has_rmc_data && strlen(data->speed) > 0) {
        GNSS_Printf(" | ?? %skm/h", data->speed);
    }
    
    GNSS_SendString("\r\n");
}

// GNSS数据处理主函数
void GNSS_ProcessData(void) {
    if (gnss_data_ready) {
        // 处理数据
        char data_copy[512];
        strncpy(data_copy, gnss_rx_buffer, sizeof(data_copy)-1);
        data_copy[sizeof(data_copy)-1] = '\0';

        GNSS_SendString("\r\n>>> 收到GNSS数据:\r\n");
        GNSS_Printf("原始数据: %s\r\n", data_copy);
        GNSS_SendString("---\r\n");

        // 手动分割两行数据
        char *first_line = data_copy;
        char *second_line = NULL;

        // 查找第二行的开始（$符号）
        char *dollar_pos = strchr(data_copy + 10, '$');  // 从第10个字符开始找$
        if (dollar_pos) {
            *dollar_pos = '\0';  // 临时终止第一行
            second_line = dollar_pos + 1;  // 第二行开始位置
        }

        int line_count = 0;
        GNSS_InitData(&gnss_data);

        // 处理第一行
        if (strlen(first_line) > 0) {
            line_count++;
            GNSS_Printf("解析第%d行: ", line_count);
            
            char nmea_sentence[256];
            if (first_line[0] != '$') {
                snprintf(nmea_sentence, sizeof(nmea_sentence), "$%s", first_line);
                GNSS_Printf("修复格式: %s - ", nmea_sentence);
            } else {
                strncpy(nmea_sentence, first_line, sizeof(nmea_sentence)-1);
                GNSS_Printf("标准格式: %s - ", nmea_sentence);
            }
            
            if (GNSS_ParseLine(nmea_sentence, &gnss_data)) {
                GNSS_SendString("成功\r\n");
            } else {
                GNSS_SendString("失败\r\n");
            }
        }

        // 处理第二行
        if (second_line && strlen(second_line) > 0) {
            line_count++;
            GNSS_Printf("解析第%d行: 标准格式: $%s - ", line_count, second_line);
            
            char nmea_sentence[256];
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$%s", second_line);
            
            if (GNSS_ParseLine(nmea_sentence, &gnss_data)) {
                GNSS_SendString("成功\r\n");
            } else {
                GNSS_SendString("失败\r\n");
            }
        }
        
        // 显示结果
        if (gnss_data.has_gga_data || gnss_data.has_rmc_data) {
            GNSS_DisplayData(&gnss_data);
        } else {
            GNSS_SendString("没有有效的GNSS数据\r\n");
        }
        
        // 重置接收状态
        gnss_data_ready = false;
        gnss_rx_index = 0;
        memset(gnss_rx_buffer, 0, sizeof(gnss_rx_buffer));
        
        GNSS_SendString("\r\n>>> 等待下一次数据...\r\n");
    }
}

// GNSS初始化函数
void GNSS_Init(void) {
    // 初始化全局变量
    memset(gnss_rx_buffer, 0, sizeof(gnss_rx_buffer));
    gnss_rx_index = 0;
    gnss_data_ready = false;
    GNSS_InitData(&gnss_data);
    
    // 启动USART1中断接收
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&gnss_rx_buffer[0], 1);
    
    GNSS_SendString("GNSS模块初始化完成，等待数据...\r\n");
}



