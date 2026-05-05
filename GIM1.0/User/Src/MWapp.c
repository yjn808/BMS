#include "MWapp.h"

MW_CONTROL_ESTIMATES MWControlAngel;
MW_RETURN_CTX wheel_return_ctx[4] = {0};

/* 电机数据接收 */
MW_MOTOR_DATA MWtestData_A;
MW_MOTOR_DATA	MWtestData_C;
MW_MOTOR_DATA	MWtestData_E;
MW_MOTOR_DATA	MWtestData_G;

/* 电机登记数据 */
MW_MOTOR_ACCESS_INFO MWtest_A = {.busId = 1, 
                               .nodeId = 1, 
                               .motorData = &MWtestData_A, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};

MW_MOTOR_ACCESS_INFO MWtest_C = {.busId = 1, 
                               .nodeId = 2, 
                               .motorData = &MWtestData_C, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};

MW_MOTOR_ACCESS_INFO MWtest_E = {.busId = 1, 
                               .nodeId = 3, 
                               .motorData = &MWtestData_E, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};

MW_MOTOR_ACCESS_INFO MWtest_G = {.busId = 1, 
                               .nodeId = 4, 
                               .motorData = &MWtestData_G, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};

/* ENDPOINT测试数据 */
MW_ENDPOINT endPointTest = {.opcode = MW_ENDPOINT_OPCODE_WRITE, 
                            .id = 95, 
                            .type = MW_ENDPOINT_DATA_TYPE_FLOAT,
                            .data.floatData = 50.0};

/* 用户自创建总线发送函数 */
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize) {
    if(busId == 0x001) CAN_Send_StdDataFrame(&hfdcan2, can_id, data);
}

/* 用户自创建总线消息函数 */
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId) {
    if(busId == 0x001) return;
}                            

void MWFunctionTest(MW_MOTOR_ACCESS_INFO MWtest, uint8_t busId, uint8_t nodeId) {
//    /* 电机总线创建 */
//    MWRegisterMotor(MWtest);
//    /* 写入线电流最大值 */
//    MWRxTxSdo(busId, nodeId, endPointTest);
//    /* 电机上电校准 */
//    MWSetAxisState(busId, nodeId, MW_AXIS_STATE_MOTOR_CALIBRATION);
//    HAL_Delay(100);
//    while(MWtest.motorData->heartBeat.currentState == MW_AXIS_STATE_MOTOR_CALIBRATION)
//        HAL_Delay(10);
//    MWSetAxisState(busId, nodeId, MW_AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
//    HAL_Delay(100);
//    while(MWtest.motorData->heartBeat.currentState == MW_AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
//        HAL_Delay(10);
#if MODE == 1    
    /* 设置模式为位置滤波控制模式 */
    MWSetControllerMode(busId, nodeId, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    /* 设置惯量值为0 */
    MWSetTrajInertia(busId, nodeId, 0);
    /* 进入闭环控制状态 */
    MWSetAxisState(busId, nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    HAL_Delay(10);
    /* 输入控制位置 */
//    MWPosControl(busId, nodeId, 2, 0, 0);
#elif MODE == 2    
    /* 设置模式为斜坡速度控制模式 */
    MWSetControllerMode(busId, nodeId, MW_VELOCITY_CONTROL, MW_RAMP_RATE_INPUT);
    /* 进入闭环控制状态 */
    MWSetAxisState(busId, nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    HAL_Delay(10);
    /* 输入控制速度 */
//    MWVelControl(busId, nodeId, 10, 0);
#endif
}

void MWMode_Init(void) {
    //注册所有4个电机
    MWRegisterMotor(MWtest_A);
    MWRegisterMotor(MWtest_C);
    MWRegisterMotor(MWtest_E);
    MWRegisterMotor(MWtest_G);
    
    //为所有电机写入线电流最大值
    MWRxTxSdo(1, 1, endPointTest);
    MWRxTxSdo(1, 2, endPointTest);
    MWRxTxSdo(1, 3, endPointTest);
    MWRxTxSdo(1, 4, endPointTest);

#if MODE == 1    
    //为所有电机设置位置滤波控制模式
    MWSetControllerMode(1, 1, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    MWSetControllerMode(1, 2, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    MWSetControllerMode(1, 3, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    MWSetControllerMode(1, 4, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    
    /* 设置惯量值为0 */
    MWSetTrajInertia(1, 1, 0);
    MWSetTrajInertia(1, 2, 0);
    MWSetTrajInertia(1, 3, 0);
    MWSetTrajInertia(1, 4, 0);
    
    /* 进入闭环控制状态 */
    MWSetAxisState(1, 1, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 2, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 3, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 4, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    HAL_Delay(10);

#elif MODE == 2    
    /* 为所有电机设置斜坡速度控制模式 */
    MWSetControllerMode(1, 1, MW_VELOCITY_CONTROL, MW_RAMP_RATE_INPUT);
    MWSetControllerMode(1, 2, MW_VELOCITY_CONTROL, MW_RAMP_RATE_INPUT);
    MWSetControllerMode(1, 3, MW_VELOCITY_CONTROL, MW_RAMP_RATE_INPUT);
    MWSetControllerMode(1, 4, MW_VELOCITY_CONTROL, MW_RAMP_RATE_INPUT);
    
    /* 进入闭环控制状态 */
    MWSetAxisState(1, 1, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 2, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 3, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 4, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
#endif
}

/* 位置模式MW电机上电车轮回正 */
MW_RETURN_STATE MWPowerOnReturnAdvanced_PosMode(
    MW_RETURN_CTX *ctx,
    MW_MOTOR_DATA *motor,
    float RETURN_ANGEL,
    uint8_t busId,
    uint8_t nodeId)
{
    const float alphaMax = 0.08f;
    const float grow     = 0.005f;
    const float POS_EPS  = 0.05f;

    float feedback = motor->encoderEstimates.encoderPosEstimate;

    /*  初始化  */
    if (!ctx->working)
    {
        ctx->working = 1;
        ctx->alpha   = 0.0f;
        ctx->timeout_start = HAL_GetTick();

        float target = RETURN_ANGEL;
        float diff   = feedback - target;

        ctx->endPoint = target;

        /* 选最近路径 */
        if (fabsf(diff) > HALF_TURN_UNITS)
        {
            if (diff > 0)
                ctx->endPoint = target + FULL_TURN_UNITS;
            else
                ctx->endPoint = target - FULL_TURN_UNITS;
        }

        return MW_RETURN_WORKING;
    }

    /* ========= 超时保护 ========= */
    if (HAL_GetTick() - ctx->timeout_start > 50000)
    {
        ctx->working = 0;
        /* 超时也发一次当前位置，防止乱跑 */
        MWPosControl(busId, nodeId, feedback, 0, 0);
        return MW_RETURN_TIMEOUT;
    }

    /* ========= 真实误差 ========= */
    float err = ctx->endPoint - feedback;

    /* ========= 到达判断（用真实编码器） ========= */
    if (fabsf(err) < POS_EPS)
    {
        ctx->working = 0;
        MWPosControl(busId, nodeId, feedback, 0, 0);
        return MW_RETURN_DONE;
    }

    /* ========= 缓动系数 ========= */
    if (ctx->alpha < alphaMax)
        ctx->alpha += grow;

    /* ========= 目标位置基于真实反馈 ========= */
    float cmd = feedback + err * ctx->alpha;

    MWPosControl(busId, nodeId, cmd, 0, 0);
    HAL_Delay(2);
    return MW_RETURN_WORKING;
}

// /* 速度模式MW电机上电车轮回正 */
// void MWPowerOnReturnAdvanced_VelMode(MW_MOTOR_DATA *MWtestData, float RETURN_ANGEL, uint8_t busId, uint8_t nodeId)
// {		
//     static int8_t working = 0;
//     static float endPoint = 0;
//     static uint32_t timeout_start = 0;
//     static float feedback = 0;
    
//     if (!working)
//     {
//         working = 1;
//         timeout_start = HAL_GetTick();
//         feedback = MWtestData->encoderEstimates.encoderPosEstimate; 
//         float target = RETURN_ANGEL; // 目标角度（单位）

//         float diff = feedback - target;
//         endPoint = target;

//         /* ----------- 计算最近方向（与位置模式相同） ----------- */
//         if (fabsf(diff) > HALF_TURN_UNITS)
//         {
//             if (diff > 0)
//                 endPoint = target + FULL_TURN_UNITS;
//             else
//                 endPoint = target - FULL_TURN_UNITS;
//         }
//         return;
//     }				
    
//     // 超时保护（8秒）
//     if (HAL_GetTick() - timeout_start > 8000) {
//         working = 0;
//         return;
//     }
 
//     feedback = MWtestData->encoderEstimates.encoderPosEstimate;
//     float error = endPoint - feedback;

//     /* ---- 到目标附近，停止 ---- */
//     if (fabsf(error) < 0.01f)
//     {
//         MWVelControl(busId, nodeId, 0, 0);  // 停止
//         working = 0;
//         return;
//     }

//     /* ---- 根据距离决定速度 ---- */
//     float vel = 0;

//     if (fabsf(error) > 1.0f)
//         vel = (error > 0) ? 3 : -3;   // 远：快速靠近
//     else
//         vel = (error > 0) ? 0.5 : -0.5;     // 近：慢速靠近

//     MWVelControl(busId, nodeId, vel, 0);
//     HAL_Delay(10);
// }

/* 统一的MW电机回正 */
void MWPowerOnReturnAll(void)
{
    // 电机数据指针数组
    MW_MOTOR_DATA* motors[4] = {&MWtestData_A, &MWtestData_C, &MWtestData_E, &MWtestData_G};
    
    // 目标角度数组
    float targets[4] = {
        MW_POWERON_RETURN_ANGEL_A,  // A轮：1.796
        MW_POWERON_RETURN_ANGEL_C,  // C轮：1.391
        MW_POWERON_RETURN_ANGEL_E,  // E轮：
        MW_POWERON_RETURN_ANGEL_G   // G轮：
    };

    // 节点ID数组
    uint8_t nodeIds[4] = {1, 2, 3, 4};

    // 逐个处理每个转向轮
    for (int i = 0; i < 4; i++) {
        MW_RETURN_STATE state = MWPowerOnReturnAdvanced_PosMode(&wheel_return_ctx[i], motors[i], 
                                                              targets[i], 1, nodeIds[i]);
        // 可以根据返回状态做进一步处理
        (void)state; // 避免编译器警告
    }
}

//设置X型锁定
void MW_SetXLock(void)
{
    float lock_angle_units = 45.0f / ANGLE_UNIT;  // 45°转换为1.0单位
    
    // X型锁定：对角轮同向 A-G对角，C-E对角
    float target_A = MW_POWERON_RETURN_ANGEL_A + lock_angle_units;   // A轮：+45°
    float target_C = MW_POWERON_RETURN_ANGEL_C - lock_angle_units;   // C轮：-45°  
    float target_E = MW_POWERON_RETURN_ANGEL_E - lock_angle_units;   // E轮：-45° (与C同向)
    float target_G = MW_POWERON_RETURN_ANGEL_G + lock_angle_units;   // G轮：+45° (与A同向)
    
    // 控制所有4个转向轮
    MWPosControl(1, 1, target_A, 0, 0);  // A轮 (节点ID=1)
    MWPosControl(1, 2, target_C, 0, 0);  // C轮 (节点ID=2)
    MWPosControl(1, 3, target_E, 0, 0);  // E轮 (节点ID=3)
    MWPosControl(1, 4, target_G, 0, 0);  // G轮 (节点ID=4)
    
    Motor_EmergencyStop();  // 停止驱动电机
}

//驱动转向MW电机 
void MotorDriver_UpdateMW(MovingWheel_t *moving_wheel)
{
    // 将方向角度转换为MW电机位置
    float target_A = MW_POWERON_RETURN_ANGEL_A + (moving_wheel->A_Dir / ANGLE_UNIT);
    float target_C = MW_POWERON_RETURN_ANGEL_C + (moving_wheel->C_Dir / ANGLE_UNIT);
    float target_E = MW_POWERON_RETURN_ANGEL_E + (moving_wheel->E_Dir / ANGLE_UNIT);
    float target_G = MW_POWERON_RETURN_ANGEL_G + (moving_wheel->G_Dir / ANGLE_UNIT);
    
    // 发送给4个MW电机
    MWPosControl(1, 1, target_A, 0, 0);  // A轮
    MWPosControl(1, 2, target_C, 0, 0);  // C轮
    MWPosControl(1, 3, target_E, 0, 0);  // E轮
    MWPosControl(1, 4, target_G, 0, 0);  // G轮
}





