/*
 * oled.h
 *
 *  Created on: 2024Äê12ÔÂ21ÈÕ
 *      Author: MAO
 */

#ifndef DSP_INCLUDE_OLED_H_
#define DSP_INCLUDE_OLED_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#define OLED_ADDRESS           0x3C //0x78 or 0x7A  default 0x78
#define MASTER_OLED_ADDRESS    0x01 // own address

#define Max_Column  128

void Init_I2CA();
Uint16 I2CA_WriteData(Uint16 Reg_Address, Uint16 OLED_Data);
Uint16 Init_OLED(void);
Uint16 OLED_Clear(void);

Uint16 OLED_ShowString(Uint16 x,Uint16 y, unsigned char *chr,Uint16 Char_Size);
Uint16 OLED_ShowChar(Uint16 x,Uint16 y,Uint16 chr,Uint16 Char_Size);
Uint16 OLED_Set_Pos(Uint16 x, Uint16 y);



#endif /* DSP_INCLUDE_OLED_H_ */
