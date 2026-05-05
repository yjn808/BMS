#ifndef __MWAPP_H
#define __MWAPP_H

#include "main.h"

#define MODE 1
#define MW_MOTOR_NUM 4
#define ANGLE_UNIT 45.0f                     // 45° = 1 单位
#define HALF_TURN_UNITS (180.0f / ANGLE_UNIT)  // = 4
#define FULL_TURN_UNITS (360.0f / ANGLE_UNIT)  // = 8


// MW电机回正目标角度
#define MW_POWERON_RETURN_ANGEL_A 1.796f    //MW_A上电回正角度
#define MW_POWERON_RETURN_ANGEL_C 3.653f    //MW_C上电回正角度 (对应原B轮)
#define MW_POWERON_RETURN_ANGEL_E 1.391f    //MW_E上电回正角度 (对应原C轮)
#define MW_POWERON_RETURN_ANGEL_G 1.345f    //MW_G上电回正角度 (需根据实际调整)


typedef struct MovingWheel_t_Struct MovingWheel_t;



/*电机回正状态判断*/
typedef enum {
    MW_RETURN_WORKING = 0,
    MW_RETURN_DONE    = 1,
    MW_RETURN_TIMEOUT = -1
} MW_RETURN_STATE;

typedef struct
{
    uint8_t  working;          // 0: idle  1: running
    float    endPoint;         // 目标点（考虑跨圈）
    float    current;          // 当前控制位置
    float    alpha;            // 缓动因子
    uint32_t timeout_start;    // 超时起点
} MW_RETURN_CTX;

// MW控制角度结构体
typedef struct {
    float MWControlAngel_A;     
    float MWControlAngel_C;     
    float MWControlAngel_E;     
    float MWControlAngel_G;     
    float MWControlAngel;       // 统一控制角度（兼容旧代码）
} MW_CONTROL_ESTIMATES;

// 全局变量声明
extern MW_CONTROL_ESTIMATES MWControlAngel;
extern MW_RETURN_CTX wheel_return_ctx[4];
extern MW_MOTOR_DATA MWtestData_A;
extern MW_MOTOR_ACCESS_INFO MWtest_A;
extern MW_MOTOR_DATA MWtestData_C;
extern MW_MOTOR_ACCESS_INFO MWtest_C;
extern MW_MOTOR_DATA MWtestData_E;
extern MW_MOTOR_ACCESS_INFO MWtest_E;
extern MW_MOTOR_DATA MWtestData_G;
extern MW_MOTOR_ACCESS_INFO MWtest_G;
extern MW_ENDPOINT endPointTest;

// 函数声明
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize);
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId);
void MWFunctionTest(MW_MOTOR_ACCESS_INFO MWtest, uint8_t busId, uint8_t nodeId);
void MWMode_Init(void);

// MW电机回正函数
MW_RETURN_STATE MWPowerOnReturnAdvanced_PosMode(MW_RETURN_CTX *ctx, MW_MOTOR_DATA *motor, float RETURN_ANGEL, uint8_t busId, uint8_t nodeId);
void MWPowerOnReturnAdvanced_VelMode(MW_MOTOR_DATA *MWtestData, float RETURN_ANGEL, uint8_t busId, uint8_t nodeId);
void MWPowerOnReturnAll(void);

// MW电机控制函数
void MW_SetXLock(void);
void MotorDriver_UpdateMW(MovingWheel_t *moving_wheel);

#endif // __MWAPP_H



