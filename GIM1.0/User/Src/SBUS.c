#include "sbus.h"


uint16_t ch[18] = {0};// 存储18个通道的解析值（SBUS支持16个比例通道+2个数字通道）

void Sbus_Parse(uint8_t *buf, uint16_t *ch)
{
    if(buf[0] != 0x0F || buf[24] != 0x00) return; // 帧头帧尾校验

    ch[0]  = ((buf[1]      | buf[2]<<8)                        & 0x07FF);
    ch[1]  = ((buf[2]>>3   | buf[3]<<5)                        & 0x07FF);
    ch[2]  = ((buf[3]>>6   | buf[4]<<2 | buf[5]<<10)           & 0x07FF);
    ch[3]  = ((buf[5]>>1   | buf[6]<<7)                        & 0x07FF);
    ch[4]  = ((buf[6]>>4   | buf[7]<<4)                        & 0x07FF);
    ch[5]  = ((buf[7]>>7   | buf[8]<<1 | buf[9]<<9)            & 0x07FF);
    ch[6]  = ((buf[9]>>2   | buf[10]<<6)                       & 0x07FF);
    ch[7]  = ((buf[10]>>5  | buf[11]<<3)                       & 0x07FF);

    ch[8]  = ((buf[12]     | buf[13]<<8)                       & 0x07FF);
    ch[9]  = ((buf[13]>>3  | buf[14]<<5)                       & 0x07FF);
    ch[10] = ((buf[14]>>6  | buf[15]<<2 | buf[16]<<10)         & 0x07FF);
    ch[11] = ((buf[16]>>1  | buf[17]<<7)                       & 0x07FF);
    ch[12] = ((buf[17]>>4  | buf[18]<<4)                       & 0x07FF);
    ch[13] = ((buf[18]>>7  | buf[19]<<1 | buf[20]<<9)          & 0x07FF);
    ch[14] = ((buf[20]>>2  | buf[21]<<6)                       & 0x07FF);
    ch[15] = ((buf[21]>>5  | buf[22]<<3)                       & 0x07FF);

    // CH16和CH17为数字通道（通常不用）
    // ch[16] = (buf[23] & 0x80) ? 1800 : 200;  // bit7
    // ch[17] = (buf[23] & 0x40) ? 1800 : 200;  // bit6
}




//C CH7: 运动模式三段开关
uint8_t Sbus_GetMoveMode(uint16_t cch4_value)
{
    if (abs((int)cch4_value - CCH4_MOVE_ACKERMANN) < CCH4_MOVE_TOLERANCE) 
        return 0;  // 阿克曼转向
    if (abs((int)cch4_value - CCH4_MOVE_CRABBING) < CCH4_MOVE_TOLERANCE) 
        return 1;  // 蟹行模式
    if (abs((int)cch4_value - CCH4_MOVE_ROTATION) < CCH4_MOVE_TOLERANCE) 
        return 2;  // 旋转模式
    
    return 0xFF;  // 无效值
}


//A CH9: 系统开关二段
uint8_t Sbus_GetSystemMode(uint16_t ach9_value)
{
    if (abs((int)ach9_value - ACH9_SYSTEM_LOCKED) < ACH9_SYSTEM_TOLERANCE) 
        return 0;  // 锁定模式
    if (abs((int)ach9_value - ACH9_SYSTEM_MOVING) < ACH9_SYSTEM_TOLERANCE) 
        return 1;  // 运动模式
    
    return 0xFF;  // 无效值
}



//  SBUS帧有效性检查
uint8_t Sbus_IsFrameValid(uint8_t *buf)
{
    if (buf[0] == SBUS_HEADER && buf[24] == SBUS_FOOTER) {
        return 1;  // 帧头帧尾正确
    }
    return 0;
}



// 将CH0转向摇杆值映射到-1.0~+1.0范围
float Sbus_MapSteering(uint16_t ch0_value)
{
    int deviation = (int)ch0_value - CH0_STEERING_CENTER;
    
    if (abs(deviation) <= CH0_STEERING_DEADZONE) 
        return 0.0f;
    
    float mapping;
    if (deviation > 0) {
        mapping = (float)(deviation - CH0_STEERING_DEADZONE) / 
                 (CH0_STEERING_MAX - CH0_STEERING_CENTER - CH0_STEERING_DEADZONE);
    } else {
        mapping = (float)(deviation + CH0_STEERING_DEADZONE) / 
                 (CH0_STEERING_CENTER - CH0_STEERING_MIN - CH0_STEERING_DEADZONE);
    }
    
    return (mapping > 1.0f) ? 1.0f : ((mapping < -1.0f) ? -1.0f : mapping);
}



// 将CH1油门摇杆值映射到-1.0~+1.0范围
float Sbus_MapThrottle(uint16_t ch1_value)
{
    int deviation = (int)ch1_value - CH1_THROTTLE_CENTER;
    
    if (abs(deviation) <= CH1_THROTTLE_DEADZONE) 
        return 0.0f;
    
    float mapping;
    if (deviation > 0) {
        mapping = (float)(deviation - CH1_THROTTLE_DEADZONE) / 
                 (CH1_THROTTLE_MAX - CH1_THROTTLE_CENTER - CH1_THROTTLE_DEADZONE);
    } else {
        mapping = (float)(deviation + CH1_THROTTLE_DEADZONE) / 
                 (CH1_THROTTLE_CENTER - CH1_THROTTLE_MIN - CH1_THROTTLE_DEADZONE);
    }
    
    return (mapping > 1.0f) ? 1.0f : ((mapping < -1.0f) ? -1.0f : mapping);
}










