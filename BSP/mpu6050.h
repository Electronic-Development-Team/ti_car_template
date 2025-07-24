#ifndef __MPU6050_H
#define __MPU6050_H
#include "myi2c.h"


typedef struct
{
    double Q_angle;    // �Ƕȹ�������Э����
    double Q_bias;     // ƫ���������Э����
    double R_measure;  // ��������Э����
    double angle;      // ��ǰ���ƽǶ�
    double bias;       // ��ǰ����ƫ��
    double P[2][2];    // ���Э�������
} Kalman_t;

typedef struct
{
    int16_t Accel_X_RAW;   // X ����ٶ�ԭʼ����
    int16_t Accel_Y_RAW;   // Y ����ٶ�ԭʼ����
    int16_t Accel_Z_RAW;   // Z ����ٶ�ԭʼ����
    double Ax;             // X ����ٶ�ֵ��g��
    double Ay;             // Y ����ٶ�ֵ��g��
    double Az;             // Z ����ٶ�ֵ��g��

    int16_t Gyro_X_RAW;    // X ��������ԭʼ����
    int16_t Gyro_Y_RAW;    // Y ��������ԭʼ����
    int16_t Gyro_Z_RAW;    // Z ��������ԭʼ����
    double Gx;             // X ����ٶ�ֵ����/s��
    double Gy;             // Y ����ٶ�ֵ����/s��
    double Gz;             // Z ����ٶ�ֵ����/s��

    float Temperature;     // ���������¶ȣ���C��

    double KalmanAngleX;   // X ��Ŀ������˲�����Ƕ�
    double KalmanAngleY;   // Y ��Ŀ������˲�����Ƕ�
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