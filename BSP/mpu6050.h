#ifndef __MPU6050_H
#define __MPU6050_H
#include "myi2c.h"


typedef struct
{
    double Q_angle;    // 角度过程噪声协方差
    double Q_bias;     // 偏差过程噪声协方差
    double R_measure;  // 测量噪声协方差
    double angle;      // 当前估计角度
    double bias;       // 当前估计偏差
    double P[2][2];    // 误差协方差矩阵
} Kalman_t;

typedef struct
{
    int16_t Accel_X_RAW;   // X 轴加速度原始数据
    int16_t Accel_Y_RAW;   // Y 轴加速度原始数据
    int16_t Accel_Z_RAW;   // Z 轴加速度原始数据
    double Ax;             // X 轴加速度值（g）
    double Ay;             // Y 轴加速度值（g）
    double Az;             // Z 轴加速度值（g）

    int16_t Gyro_X_RAW;    // X 轴陀螺仪原始数据
    int16_t Gyro_Y_RAW;    // Y 轴陀螺仪原始数据
    int16_t Gyro_Z_RAW;    // Z 轴陀螺仪原始数据
    double Gx;             // X 轴角速度值（°/s）
    double Gy;             // Y 轴角速度值（°/s）
    double Gz;             // Z 轴角速度值（°/s）

    float Temperature;     // 传感器的温度（°C）

    double KalmanAngleX;   // X 轴的卡尔曼滤波计算角度
    double KalmanAngleY;   // Y 轴的卡尔曼滤波计算角度
} MPU6050_t;






void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
 
void MPU6050_Init(int gyro,int accel);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(MPU6050_t *DataStruct);

float MPU6050_ConvertAccel(int16_t value, uint8_t range);
float MPU6050_ConvertGyro(int16_t value, uint8_t range);
float MPU6050_ConvertTemp(int16_t value);

void MPU6050_ReadSensors(MPU6050_t *DataStruct) ;
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);



#endif