#include "ti_msp_dl_config.h"
#include "myi2c.h"
#include "mpu6050_Reg.h"
 #include "math.h"
 #include "mpu6050.h"
 #include <stdint.h>
 
#define PI 3.1415926535f
#define RtA 57.2957795f  // 弧度转角度
#define AtR 0.0174532925f  // 角度转弧度

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址
 
#define Gyro_Range 0x18
#define Accel_Range 0x00

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


/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}
 
/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	  
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}
 
/**
  * 函    数：MPU6050初始化
	* 参    数：int gyro角加速度量程  0:250	1:500	2:1000	3:2000      
int accel加速度量程   0:2g	1;4g	2:8g	3:16g
  * 返 回 值：无
  */
void MPU6050_Init(int gyro,int accel)
{
	MyI2C_Init();									//先初始化底层的I2C
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	switch(gyro)
	{
		case 0:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);	//陀螺仪配置寄存器，选择满量程为±250°/s
			break;
		case 1:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);	//陀螺仪配置寄存器，选择满量程为±500°/s
			break;
		case 2:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10);	//陀螺仪配置寄存器，选择满量程为±1000°/s
			break;
		case 3:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
			break;
	}
	switch(accel)
	{
		case 0:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);//加速度计配置寄存器，选择满量程为±2g
			break;
		case 1:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x08);	//加速度计配置寄存器，选择满量程为±4g
			break;
		case 2:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x10);	//加速度计配置寄存器，选择满量程为±8g
			break;
		case 3:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
			break;
	}
}
 
/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}
 
/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(MPU6050_t *DataStruct)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	DataStruct->Accel_X_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	DataStruct->Accel_Y_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	DataStruct->Accel_Z_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	DataStruct->Gyro_X_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	DataStruct->Gyro_Y_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	DataStruct->Gyro_Z_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);		//读取陀螺仪Z轴的低8位数据
	DataStruct->Temperature = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
}


float MPU6050_ConvertAccel(int16_t value, uint8_t range) 
{
    float factor; // 用于存储转换因子
    // 根据加速度计的量程选择合适的转换因子
    switch (range) {
        case 0x00: factor = 16384.0; break; // ±2g，转换因子为16384
        case 0x08: factor = 8192.0; break;  // ±4g，转换因子为8192
        case 0x10: factor = 4096.0; break;  // ±8g，转换因子为4096
        case 0x18: factor = 2048.0; break;  // ±16g，转换因子为2048
        default: factor = 16384.0; break;    // 默认量程范围为±2g
    }
    // 将原始加速度值转换为g单位
    return value / factor;
}
 
float MPU6050_ConvertGyro(int16_t value, uint8_t range) 
{
    float factor; // 用于存储转换因子
    // 根据陀螺仪的量程选择合适的转换因子
    switch (range) {
        case 0x00: factor = 131.0; break;  // ±250°/s，转换因子为131
        case 0x08: factor = 65.5; break;   // ±500°/s，转换因子为65.5
        case 0x10: factor = 32.8; break;   // ±1000°/s，转换因子为32.8
        case 0x18: factor = 16.4; break;   // ±2000°/s，转换因子为16.4
        default: factor = 131.0; break;     // 默认量程范围为±250°/s
    }
    // 将原始陀螺仪值转换为°/s单位
    return value / factor;
}

//温度数据转化
float MPU6050_ConvertTemp(int16_t value) 
{
    value = (float)((int16_t)value / (float)340.0 + (float)36.53);
    return value ;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
/*---------------------预测阶段--------------------------*/

    // 1. 预测角度
    // 角速度 = 陀螺仪角速度 - 陀螺仪偏置值 (得到无偏角速度)
    double rate = newRate - Kalman->bias;
    // 预测角度 = 前一时刻角速 + 时间间隔*角速度
    Kalman->angle += dt * rate;

    // 2. 预测协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // 预测角度协方差
    Kalman->P[0][1] -= dt * Kalman->P[1][1];  // 预测角度和偏置的协方差
    Kalman->P[1][0] -= dt * Kalman->P[1][1];  // 预测偏置和角度的协方差
    Kalman->P[1][1] += Kalman->Q_bias * dt;   // 预测偏置协方差


/*---------------------更新阶段--------------------------*/

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
}

void MPU6050_ReadSensors(MPU6050_t *DataStruct) 
{
	unsigned long count;
	MPU6050_GetData(DataStruct);// 读取加速度计和陀螺仪的原始数据

    // 将加速度计的原始数据转换为浮点数形式（单位：g）
    DataStruct->Ax = MPU6050_ConvertAccel(DataStruct->Accel_X_RAW, Accel_Range); // ±2g量程
    DataStruct->Ay = MPU6050_ConvertAccel(DataStruct->Accel_Y_RAW, Accel_Range); // ±2g量程
    DataStruct->Az = MPU6050_ConvertAccel(DataStruct->Accel_Z_RAW, Accel_Range); // ±2g量程
    // 将陀螺仪的原始数据转换为浮点数形式（单位：°/s）
    DataStruct->Gx = MPU6050_ConvertGyro(DataStruct->Gyro_X_RAW, Gyro_Range)*AtR; // ±250°/s量程
    DataStruct->Gy = MPU6050_ConvertGyro(DataStruct->Gyro_Y_RAW, Gyro_Range)*AtR; // ±250°/s量程
    DataStruct->Gz = MPU6050_ConvertGyro(DataStruct->Gyro_Z_RAW, Gyro_Range)*AtR; // ±250°/s量程
		//温度
		DataStruct->Temperature = MPU6050_ConvertTemp(DataStruct->Temperature);
		mspm0_get_clock_ms(&count);
		double dt = (double)( count - timer) / 1000;  // 获取时间差（毫秒），转换为秒
    mspm0_get_clock_ms(&count);  // 更新计时器
		timer = count;
	// 计算滚转角 roll
    double roll;  // 用于存储计算得到的滚转角（X 轴）
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RtA;  // 先计算出弧度制 roll ，再弧度转换为角度值
    }
    else
    {
        roll = 0.0;
    }
	
	// 计算俯仰角 pitch
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RtA;

    // 如果俯仰角度变化过快(超过90度)，防止角度跳变
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch; 
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        // 卡尔曼滤波器更新俯仰角度 Y
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    // 如果俯仰角绝对值超过 90 度，则反转 X 轴的陀螺仪角速度，防止符号错误
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;

    // 卡尔曼滤波器更新滚转角度 X
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

}


