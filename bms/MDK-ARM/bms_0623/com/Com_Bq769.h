#ifndef __COM_BQ769_H_
#define __COM_BQ769_H_
#include "stdint.h"
/**
 * @brief 通过查表法把电阻换算为温度
 */
int8_t Com_BQ769_getTemperByResist(uint32_t resistance);

/**
 * @brief 通过查表法把电压换算为电量百分比
 */
uint8_t Com_BQ769_getPercentByVoltage(uint16_t voltage);

#endif // __COM_BQ769_H_
