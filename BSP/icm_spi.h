#ifndef _ICM_SPI_H__
#define _ICM_SPI_H__

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "math.h"
#include <stdint.h>

//CS引脚的输出控制
//x=0时输出低电平
//x=1时输出高电平
#define SPI_CS(x)  ( (x) ? DL_GPIO_setPins(CS_PORT,CS_PIN_PIN) : DL_GPIO_clearPins(CS_PORT,CS_PIN_PIN) )

#define ICM_ADDRESS       0X68
#define ICM_WHOAMI    0X75
#define ICM_DEVICE_CONFIG    0X11
#define ICM_PWR_MGMT0     0x4E
#define ICM_GYRO_CONFIG0  0x4F
#define ICM_ACCEL_CONFIG0 0x50
#define ICM_SENSOR_CONFIG0 0x03
#define ICM_ACCEL_DATA_X1 0x1F
#define ICM_ACCEL_DATA_X0 0x20
#define ICM_ACCEL_DATA_Y1 0x21
#define ICM_ACCEL_DATA_Y0 0x22
#define ICM_ACCEL_DATA_Z1 0x23
#define ICM_ACCEL_DATA_Z0 0x24
#define ICM_GYRO_DATA_X1  0x25
#define ICM_GYRO_DATA_X0  0x26
#define ICM_GYRO_DATA_Y1  0x27
#define ICM_GYRO_DATA_Y0  0x28
#define ICM_GYRO_DATA_Z1  0x29
#define ICM_GYRO_DATA_Z0  0x2A
#define ICM_REG_BANK_SEL  0x76
#define ICM_TEMP_DATA1  0x1D
#define ICM_TEMP_DATA0  0x1E


#define ICM_GYRO_RANGE  0x00  //0x00:±2000dps（默认）0x20:±1000dps 0x40:±500dps 0x60:±250dps
#define ICM_ACCEL_RANGE 0x00 //0x00:±16g（默认）0x20:±8g 0x40:±4g 0x60:±2g
#define PI 3.1415926535f
#define RtA 57.2957795f  // 弧度转角度
#define AtR 0.0174532925f  // 角度转弧度

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
} ICM_t;


uint8_t spi_read_write_byte(uint8_t dat);
uint8_t spi_read_byte(uint8_t addr);
uint8_t spi_write_byte(uint8_t addr,uint8_t data);
uint8_t icm_readID(void);//读取icm42688的ID
void icm_init(void);
void ICM_ReadData(ICM_t *DataStruct);
float ICM_ConvertAccel(int16_t value, uint8_t range) ;
float ICM_ConvertGyro(int16_t value, uint8_t range) ;
float ICM_ConvertTemp(int16_t value);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void ICM_ReadSensors(ICM_t *DataStruct);

//void W25Q128_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte);      //W25Q128写数据
//void W25Q128_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length);//W25Q128读数据
#endif