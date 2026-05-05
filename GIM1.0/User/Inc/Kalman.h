#ifndef _KALMAN_FIXED_H_
#define _KALMAN_FIXED_H_

#include <stdint.h>

// ========== 卡尔曼滤波器结构体定义 ==========
typedef struct {
    // 噪声参数
    float Q_angle;          // 角度过程噪声方差
    float Q_bias;           // 陀螺仪零偏过程噪声方差
    float R_measure;        // 测量噪声方差（加速度计）
    
    // 状态变量
    float angle;            // 滤波后的角度估计值
    float bias;             // 陀螺仪零偏估计值
    float rate;             // 去偏后的角速度
    
    // 误差协方差矩阵 (2x2)
    float P[2][2];          // P[0][0]: 角度方差
                           // P[0][1]: 角度-偏置协方差
                           // P[1][0]: 偏置-角度协方差
                           // P[1][1]: 偏置方差
} KalmanFilter_t;

// ========== 预设参数配置 ==========
// 快速响应配置（适用于动态环境）
#define KALMAN_FAST_Q_ANGLE     0.01f
#define KALMAN_FAST_Q_BIAS      0.01f
#define KALMAN_FAST_R_MEASURE   0.1f

// 平滑配置（适用于静态环境）
#define KALMAN_SMOOTH_Q_ANGLE   0.001f
#define KALMAN_SMOOTH_Q_BIAS    0.003f
#define KALMAN_SMOOTH_R_MEASURE 0.03f

// 默认配置（平衡响应速度和平滑度）
#define KALMAN_DEFAULT_Q_ANGLE  0.005f
#define KALMAN_DEFAULT_Q_BIAS   0.005f
#define KALMAN_DEFAULT_R_MEASURE 0.05f

// ========== 函数声明 ==========

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 */
void Kalman_Init(KalmanFilter_t *kf);

/**
 * @brief 设置卡尔曼滤波器参数
 * @param kf 卡尔曼滤波器指针
 * @param Q_angle 角度过程噪声
 * @param Q_bias 偏置过程噪声
 * @param R_measure 测量噪声
 */
void Kalman_SetParameters(KalmanFilter_t *kf, float Q_angle, float Q_bias, float R_measure);

/**
 * @brief 卡尔曼滤波器状态更新
 * @param kf 卡尔曼滤波器指针
 * @param newAngle 新的角度测量值（来自加速度计）
 * @param newRate 新的角速度测量值（来自陀螺仪）
 * @param dt 时间间隔（秒）
 * @return 滤波后的角度估计值
 */
float Kalman_Update(KalmanFilter_t *kf, float newAngle, float newRate, float dt);

/**
 * @brief 获取当前偏置估计值
 * @param kf 卡尔曼滤波器指针
 * @return 当前偏置估计值
 */
float Kalman_GetBias(KalmanFilter_t *kf);

/**
 * @brief 获取当前角速度估计值
 * @param kf 卡尔曼滤波器指针
 * @return 当前角速度估计值（已去偏）
 */
float Kalman_GetRate(KalmanFilter_t *kf);

/**
 * @brief 重置卡尔曼滤波器状态
 * @param kf 卡尔曼滤波器指针
 */
void Kalman_Reset(KalmanFilter_t *kf);

/**
 * @brief 设置初始角度
 * @param kf 卡尔曼滤波器指针
 * @param angle 初始角度值
 */
void Kalman_SetAngle(KalmanFilter_t *kf, float angle);

// ========== 参数配置宏函数 ==========
/**
 * @brief 快速配置滤波器为快速响应模式
 */
#define KALMAN_CONFIG_FAST(kf) \
    Kalman_SetParameters(kf, KALMAN_FAST_Q_ANGLE, KALMAN_FAST_Q_BIAS, KALMAN_FAST_R_MEASURE)

/**
 * @brief 快速配置滤波器为平滑模式
 */
#define KALMAN_CONFIG_SMOOTH(kf) \
    Kalman_SetParameters(kf, KALMAN_SMOOTH_Q_ANGLE, KALMAN_SMOOTH_Q_BIAS, KALMAN_SMOOTH_R_MEASURE)

/**
 * @brief 快速配置滤波器为默认模式
 */
#define KALMAN_CONFIG_DEFAULT(kf) \
    Kalman_SetParameters(kf, KALMAN_DEFAULT_Q_ANGLE, KALMAN_DEFAULT_Q_BIAS, KALMAN_DEFAULT_R_MEASURE)

#endif /* _KALMAN_FIXED_H_ */


