#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

#define MAX_SPEEDSTEP		5

typedef struct
{
	uint32_t flag;
	uint32_t Z;
	float angle;
	float angle_mach;
	float speed;
	float speed_filter;
	float temp;
	uint32_t count;
	uint32_t dir;
} Encoder_Typedef;


extern Encoder_Typedef Encoder;

void Encoder_Init(void);
void Encoder_ZeroAlignment(void);	
void Encoder_SpeedFilter(void);
void Encoder_SpeedCalc(void);
void BubbleSort(float* speed, uint8_t length);
void Encoder_SpeedRamp(void);
#endif



