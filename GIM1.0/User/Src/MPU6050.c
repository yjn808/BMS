#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105		//用以弧度转角度，角度 = 弧度 × (180/π)，其中 180/π 的值约为 57.2958

#define WHO_AM_I_REG 				0x75				// MPU6050 的 WHO_AM_I 寄存器地址。通过读取该寄存器的值，如果返回 0x68，则表示该 I2C 设备是 MPU6050 传感器。
#define PWR_MGMT_1_REG 				0x6B  			// MPU6050 的 Power Management 1 寄存器地址 (电源管理 1 寄存器)，用于配置电源模式、时钟源选择、设备复位和睡眠模式等功能。
#define SMPLRT_DIV_REG 				0x19     		// MPU6050 的 Sample Rate Divider 寄存器地址 (采样速率分频器)，通过该寄存器设置分频系数，配置 MPU6050 数据输出速率。
#define ACCEL_CONFIG_REG 			0x1C     		// MPU6050 的 ACCEL_CONFIG 寄存器地址 (加速度计配置寄存器)，用于设置加速度计的量程范围（例如 ±2g、±4g 等）。
#define ACCEL_XOUT_H_REG 			0x3B     		// MPU6050 的加速度计 X 轴高位数据寄存器地址，用于读取 X 轴加速度高字节数据。
#define TEMP_OUT_H_REG 				0x41      	// MPU6050 的温度传感器高位数据寄存器地址，用于读取温度的高字节数据。
#define GYRO_CONFIG_REG 			0x1B      	// MPU6050 的 GYRO_CONFIG 寄存器地址 (陀螺仪配置寄存器)，用于设置陀螺仪的量程范围（例如 ±250°/s、±500°/s 等）。
#define GYRO_XOUT_H_REG 			0x43      	// MPU6050 的陀螺仪 X 轴高位数据寄存器地址，用于读取 X 轴陀螺仪角速度的高字节数据。



int16_t AccX = 0, AccY = 0, AccZ = 0;       // 加速度计原始数据全局变量
int16_t GyroX = 0, GyroY = 0, GyroZ = 0;    // 陀螺仪原始数据全局变量
static float mpu_temperature = 0.0f;        // 温度全局变量

// Setup MPU6050
#define MPU6050_ADDR 0xD0  		// MPU6050 的补位后的 8 位地址，MPU6050的 7 位地址为 110 100x (0x68 或 0x69)，最低位 x 取决于 AD0 引脚状态。I2C 通信时，左移一位并在最低位加 0（写）或 1（读），形成 8 位地址。以写为例，MPU6050 地址为 1101 0000 (0xD0) 表示写操作
const uint16_t i2c_timeout = 100;  // I2C 通信超时时间，单位为毫秒。
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
		uint8_t check;		// 存储从 WHO_AM_I 寄存器读取到的设备 ID，实际上是该寄存器的值0x68（该寄存器默认值为 0x68 (104)，也即 MPU6050 的地址
    uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);		// 通过 I2Cx 访问 MPU6050 的 WHO_AM_I 寄存器，读取 1 字节数据到 check，超时时间为 i2c_timeout。


	if (check == 104) //检验check的值是否为0x68
    {
			// 设置Power Management 1 寄存器 (电源管理1寄存器)，将其第7位置1 (1000 0000) 进行设备复位，防止数据残留
        Data = 0x80;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
			
			// 设置 Power Management 1 寄存器 (电源管理1寄存器)，将其所有位置0 (0000 0000) ，进行唤醒
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // 设置采样速率分频器，50Hz采样率 , 8 K / ( 1 + 159 ) = 50 Hz   计算公式 采样频率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV)
        Data = 159;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);
			
			//设置加速度计配置，范围 +/-2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);
				
			// 设置陀螺仪配置，范围 +/-500°/s
        Data = 0x08;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
			
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; 													// 定义存储从 MPU6050 读取的 6 个字节加速度计数据

    // 读取加速度计寄存器
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 	// X轴加速度计原始数据，两字节拼接
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); 	// Y轴加速度计原始数据
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); 	// Z轴加速度计原始数据

    /*** 转换为重力单位 ***/
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; 				// X 轴加速度值，±2g量程下，灵敏度为16384 LSB/g
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0; 				// Y 轴加速度值
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector; 		// Z 轴加速度值，使用自定义校准系数
    
    // 更新全局变量
    AccX = DataStruct->Accel_X_RAW;
    AccY = DataStruct->Accel_Y_RAW;
    AccZ = DataStruct->Accel_Z_RAW;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 读取陀螺仪寄存器
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 	// X轴陀螺仪原始数据
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); 	// Y轴陀螺仪原始数据
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); 	// Z轴陀螺仪原始数据

    /*** 转换为角速度单位 ***/
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 65.5; 		// ±500°/s 量程下，灵敏度为 65.5 LSB/(°/s)
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 65.5; 		// 陀螺仪 Y 轴角速度，单位：°/s
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 65.5; 		// 陀螺仪 Z 轴角速度，单位：°/s
    
    // 更新全局变量
    GyroX = DataStruct->Gyro_X_RAW;
    GyroY = DataStruct->Gyro_Y_RAW;
    GyroZ = DataStruct->Gyro_Z_RAW;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // 读取温度寄存器数据
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 						// 将两个字节合并成一个 16 位值
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); 	// 温度转换公式: Temperature = TEMP_OUT/340 + 36.53
    
    // 更新全局变量
    mpu_temperature = DataStruct->Temperature;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // 读取从 ACCEL_XOUT_H 开始的 14 字节，包含所有加速度计、温度、陀螺仪数据
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 	// 加速度计 X 轴原始数据
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); 	// 加速度计 Y 轴原始数据
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); 	// 加速度计 Z 轴原始数据
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]); 						// 温度原始数据
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]); 	// 陀螺仪 X 轴原始数据
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]); 	// 陀螺仪 Y 轴原始数据
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]); 	// 陀螺仪 Z 轴原始数据

    /*** 将原始数据转换为物理量 ***/
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;    // X轴加速度，±2g量程下，灵敏度为16384 LSB/g
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;    // Y轴加速度
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;    // Z轴加速度，使用自定义校准系数

    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);  // 温度，转换公式: Temperature = TEMP_OUT/340 + 36.53

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 65.5;        // X轴角速度，±500°/s量程下，灵敏度为65.5 LSB/(°/s)
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 65.5;        // Y轴角速度
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 65.5;        // Z轴角速度
    
    // 更新全局变量
    AccX = DataStruct->Accel_X_RAW;
    AccY = DataStruct->Accel_Y_RAW;
    AccZ = DataStruct->Accel_Z_RAW;
    GyroX = DataStruct->Gyro_X_RAW;
    GyroY = DataStruct->Gyro_Y_RAW;
    GyroZ = DataStruct->Gyro_Z_RAW;
    mpu_temperature = DataStruct->Temperature;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
/*---------------------预测阶段--------------------------*/

    // 1. 预测角度：角度 = 上次角度 + (角速度 - 偏置) * 时间
    double rate = newRate - Kalman->bias;  // 去除偏置后的角速度（使用局部变量）
    Kalman->angle += dt * rate;            // 预测的角度

    // 2. 预测协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // 预测角度协方差
    Kalman->P[0][1] -= dt * Kalman->P[1][1];  // 预测角度和偏置的协方差
    Kalman->P[1][0] -= dt * Kalman->P[1][1];  // 预测偏置和角度的协方差
    Kalman->P[1][1] += Kalman->Q_bias * dt;   // 预测偏置协方差



    // 3. 更新卡尔曼增益
    // 总误差协方差 = 预测协方差 + 测量噪声协方差
    double S = Kalman->P[0][0] + Kalman->R_measure;
    // 卡尔曼增益 K
    double K[2]; 
    K[0] = Kalman->P[0][0] / S;  // 角度的卡尔曼增益
    K[1] = Kalman->P[1][0] / S;  // 偏置的卡尔曼增益

    // 4. 更新角度和偏置
    // 测量残差 = 测量值 - 预测值
    double y = newAngle - Kalman->angle;
    // 根据卡尔曼增益，更新角度和偏置的估计值，修正预测阶段的误差
    Kalman->angle += K[0] * y;  // 更新角度估计。
    Kalman->bias += K[1] * y;   // 更新偏置估计

    // 5. 更新协方差矩阵 P
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;  // 更新角度协方差
    Kalman->P[0][1] -= K[0] * P01_temp;  // 更新角度和偏置的协方差
    Kalman->P[1][0] -= K[1] * P00_temp;  // 更新偏置和角度的协方差
    Kalman->P[1][1] -= K[1] * P01_temp;  // 更新偏置协方差

    // 6. 返回滤波后的最优角度值
    return Kalman->angle;
};

// ========== 为mavlink_app.c提供的函数实现 ==========

// 获取MPU6050温度（为mavlink_app.c提供接口）
float MPU_Get_Temperature(void)
{
    return mpu_temperature;
}

// 更新全局变量（从结构体数据更新到全局变量）
void MPU6050_UpdateGlobalVars(MPU6050_t *DataStruct)
{
    if (DataStruct != ((void*)0)) {
        AccX = DataStruct->Accel_X_RAW;
        AccY = DataStruct->Accel_Y_RAW;
        AccZ = DataStruct->Accel_Z_RAW;
        GyroX = DataStruct->Gyro_X_RAW;
        GyroY = DataStruct->Gyro_Y_RAW;
        GyroZ = DataStruct->Gyro_Z_RAW;
        mpu_temperature = DataStruct->Temperature;
    }
}








