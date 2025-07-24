#include "ti_msp_dl_config.h"
#include "myi2c.h"
#include "mpu6050_Reg.h"
 #include "math.h"
 #include "mpu6050.h"
 #include <stdint.h>
 
#define PI 3.1415926535f
#define RtA 57.2957795f  // ����ת�Ƕ�
#define AtR 0.0174532925f  // �Ƕ�ת����

#define MPU6050_ADDRESS		0xD0		//MPU6050��I2C�ӻ���ַ
 
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
  * ��    ����MPU6050д�Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * ��    ����Data Ҫд��Ĵ��������ݣ���Χ��0x00~0xFF
  * �� �� ֵ����
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C��ʼ
	MyI2C_SendByte(MPU6050_ADDRESS);	//���ʹӻ���ַ����дλΪ0����ʾ����д��
	MyI2C_ReceiveAck();					//����Ӧ��
	MyI2C_SendByte(RegAddress);			//���ͼĴ�����ַ
	MyI2C_ReceiveAck();					//����Ӧ��
	MyI2C_SendByte(Data);				//����Ҫд��Ĵ���������
	MyI2C_ReceiveAck();					//����Ӧ��
	MyI2C_Stop();						//I2C��ֹ
}
 
/**
  * ��    ����MPU6050���Ĵ���
  * ��    ����RegAddress �Ĵ�����ַ����Χ���ο�MPU6050�ֲ�ļĴ�������
  * �� �� ֵ����ȡ�Ĵ��������ݣ���Χ��0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	  
	MyI2C_Start();						//I2C��ʼ
	MyI2C_SendByte(MPU6050_ADDRESS);	//���ʹӻ���ַ����дλΪ0����ʾ����д��
	MyI2C_ReceiveAck();					//����Ӧ��
	MyI2C_SendByte(RegAddress);			//���ͼĴ�����ַ
	MyI2C_ReceiveAck();					//����Ӧ��
	
	MyI2C_Start();						//I2C�ظ���ʼ
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//���ʹӻ���ַ����дλΪ1����ʾ������ȡ
	MyI2C_ReceiveAck();					//����Ӧ��
	Data = MyI2C_ReceiveByte();			//����ָ���Ĵ���������
	MyI2C_SendAck(1);					//����Ӧ�𣬸��ӻ���Ӧ����ֹ�ӻ����������
	MyI2C_Stop();						//I2C��ֹ
	
	return Data;
}
 
/**
  * ��    ����MPU6050��ʼ��
	* ��    ����int gyro�Ǽ��ٶ�����  0:250	1:500	2:1000	3:2000      
int accel���ٶ�����   0:2g	1;4g	2:8g	3:16g
  * �� �� ֵ����
  */
void MPU6050_Init(int gyro,int accel)
{
	MyI2C_Init();									//�ȳ�ʼ���ײ��I2C
	
	/*MPU6050�Ĵ�����ʼ������Ҫ����MPU6050�ֲ�ļĴ����������ã��˴��������˲�����Ҫ�ļĴ���*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//��Դ����Ĵ���1��ȡ��˯��ģʽ��ѡ��ʱ��ԴΪX��������
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//��Դ����Ĵ���2������Ĭ��ֵ0���������������
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//�����ʷ�Ƶ�Ĵ��������ò�����
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//���üĴ���������DLPF
	switch(gyro)
	{
		case 0:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);	//���������üĴ�����ѡ��������Ϊ��250��/s
			break;
		case 1:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);	//���������üĴ�����ѡ��������Ϊ��500��/s
			break;
		case 2:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10);	//���������üĴ�����ѡ��������Ϊ��1000��/s
			break;
		case 3:
			MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//���������üĴ�����ѡ��������Ϊ��2000��/s
			break;
	}
	switch(accel)
	{
		case 0:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);//���ٶȼ����üĴ�����ѡ��������Ϊ��2g
			break;
		case 1:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x08);	//���ٶȼ����üĴ�����ѡ��������Ϊ��4g
			break;
		case 2:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x10);	//���ٶȼ����üĴ�����ѡ��������Ϊ��8g
			break;
		case 3:
			MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//���ٶȼ����üĴ�����ѡ��������Ϊ��16g
			break;
	}
}
 
/**
  * ��    ����MPU6050��ȡID��
  * ��    ������
  * �� �� ֵ��MPU6050��ID��
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//����WHO_AM_I�Ĵ�����ֵ
}
 
/**
  * ��    ����MPU6050��ȡ����
  * ��    ����AccX AccY AccZ ���ٶȼ�X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * ��    ����GyroX GyroY GyroZ ������X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
  * �� �� ֵ����
  */
void MPU6050_GetData(MPU6050_t *DataStruct)
{
	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
	DataStruct->Accel_X_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//��ȡ���ٶȼ�Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//��ȡ���ٶȼ�Y��ĵ�8λ����
	DataStruct->Accel_Y_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//��ȡ���ٶȼ�Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//��ȡ���ٶȼ�Z��ĵ�8λ����
	DataStruct->Accel_Z_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//��ȡ������X��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//��ȡ������X��ĵ�8λ����
	DataStruct->Gyro_X_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//��ȡ������Y��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//��ȡ������Y��ĵ�8λ����
	DataStruct->Gyro_Y_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
	DataStruct->Gyro_Z_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);		//��ȡ������Z��ĸ�8λ����
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);		//��ȡ������Z��ĵ�8λ����
	DataStruct->Temperature = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
}


float MPU6050_ConvertAccel(int16_t value, uint8_t range) 
{
    float factor; // ���ڴ洢ת������
    // ���ݼ��ٶȼƵ�����ѡ����ʵ�ת������
    switch (range) {
        case 0x00: factor = 16384.0; break; // ��2g��ת������Ϊ16384
        case 0x08: factor = 8192.0; break;  // ��4g��ת������Ϊ8192
        case 0x10: factor = 4096.0; break;  // ��8g��ת������Ϊ4096
        case 0x18: factor = 2048.0; break;  // ��16g��ת������Ϊ2048
        default: factor = 16384.0; break;    // Ĭ�����̷�ΧΪ��2g
    }
    // ��ԭʼ���ٶ�ֵת��Ϊg��λ
    return value / factor;
}
 
float MPU6050_ConvertGyro(int16_t value, uint8_t range) 
{
    float factor; // ���ڴ洢ת������
    // ���������ǵ�����ѡ����ʵ�ת������
    switch (range) {
        case 0x00: factor = 131.0; break;  // ��250��/s��ת������Ϊ131
        case 0x08: factor = 65.5; break;   // ��500��/s��ת������Ϊ65.5
        case 0x10: factor = 32.8; break;   // ��1000��/s��ת������Ϊ32.8
        case 0x18: factor = 16.4; break;   // ��2000��/s��ת������Ϊ16.4
        default: factor = 131.0; break;     // Ĭ�����̷�ΧΪ��250��/s
    }
    // ��ԭʼ������ֵת��Ϊ��/s��λ
    return value / factor;
}

//�¶�����ת��
float MPU6050_ConvertTemp(int16_t value) 
{
    value = (float)((int16_t)value / (float)340.0 + (float)36.53);
    return value ;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
/*---------------------Ԥ��׶�--------------------------*/

    // 1. Ԥ��Ƕ�
    // ���ٶ� = �����ǽ��ٶ� - ������ƫ��ֵ (�õ���ƫ���ٶ�)
    double rate = newRate - Kalman->bias;
    // Ԥ��Ƕ� = ǰһʱ�̽��� + ʱ����*���ٶ�
    Kalman->angle += dt * rate;

    // 2. Ԥ��Э�������
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // Ԥ��Ƕ�Э����
    Kalman->P[0][1] -= dt * Kalman->P[1][1];  // Ԥ��ǶȺ�ƫ�õ�Э����
    Kalman->P[1][0] -= dt * Kalman->P[1][1];  // Ԥ��ƫ�úͽǶȵ�Э����
    Kalman->P[1][1] += Kalman->Q_bias * dt;   // Ԥ��ƫ��Э����


/*---------------------���½׶�--------------------------*/

    // 3. ���¿���������
    // �����Э���� = Ԥ��Э���� + ��������Э����
    double S = Kalman->P[0][0] + Kalman->R_measure;
    // ���������� K
    double K[2]; 
    K[0] = Kalman->P[0][0] / S;  // �ǶȵĿ���������
    K[1] = Kalman->P[1][0] / S;  // ƫ�õĿ���������

    // 4. ���½ǶȺ�ƫ��
    // �����в� = ����ֵ - Ԥ��ֵ
    double y = newAngle - Kalman->angle;
    // ���ݿ��������棬���½ǶȺ�ƫ�õĹ���ֵ������Ԥ��׶ε����
    Kalman->angle += K[0] * y;  // ���½Ƕȹ��ơ�
    Kalman->bias += K[1] * y;   // ����ƫ�ù���

    // 5. ����Э������� P
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;  // ���½Ƕ�Э����
    Kalman->P[0][1] -= K[0] * P01_temp;  // ���½ǶȺ�ƫ�õ�Э����
    Kalman->P[1][0] -= K[1] * P00_temp;  // ����ƫ�úͽǶȵ�Э����
    Kalman->P[1][1] -= K[1] * P01_temp;  // ����ƫ��Э����

    // 6. �����˲�������ŽǶ�ֵ
    return Kalman->angle;
}

void MPU6050_ReadSensors(MPU6050_t *DataStruct) 
{
	unsigned long count;
	MPU6050_GetData(DataStruct);// ��ȡ���ٶȼƺ������ǵ�ԭʼ����

    // �����ٶȼƵ�ԭʼ����ת��Ϊ��������ʽ����λ��g��
    DataStruct->Ax = MPU6050_ConvertAccel(DataStruct->Accel_X_RAW, Accel_Range); // ��2g����
    DataStruct->Ay = MPU6050_ConvertAccel(DataStruct->Accel_Y_RAW, Accel_Range); // ��2g����
    DataStruct->Az = MPU6050_ConvertAccel(DataStruct->Accel_Z_RAW, Accel_Range); // ��2g����
    // �������ǵ�ԭʼ����ת��Ϊ��������ʽ����λ����/s��
    DataStruct->Gx = MPU6050_ConvertGyro(DataStruct->Gyro_X_RAW, Gyro_Range)*AtR; // ��250��/s����
    DataStruct->Gy = MPU6050_ConvertGyro(DataStruct->Gyro_Y_RAW, Gyro_Range)*AtR; // ��250��/s����
    DataStruct->Gz = MPU6050_ConvertGyro(DataStruct->Gyro_Z_RAW, Gyro_Range)*AtR; // ��250��/s����
		//�¶�
		DataStruct->Temperature = MPU6050_ConvertTemp(DataStruct->Temperature);
		mspm0_get_clock_ms(&count);
		double dt = (double)( count - timer) / 1000;  // ��ȡʱ�����룩��ת��Ϊ��
    mspm0_get_clock_ms(&count);  // ���¼�ʱ��
		timer = count;
	// �����ת�� roll
    double roll;  // ���ڴ洢����õ��Ĺ�ת�ǣ�X �ᣩ
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RtA;  // �ȼ���������� roll ���ٻ���ת��Ϊ�Ƕ�ֵ
    }
    else
    {
        roll = 0.0;
    }
	
	// ���㸩���� pitch
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RtA;

    // ��������Ƕȱ仯����(����90��)����ֹ�Ƕ�����
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch; 
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        // �������˲������¸����Ƕ� Y
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    // ��������Ǿ���ֵ���� 90 �ȣ���ת X ��������ǽ��ٶȣ���ֹ���Ŵ���
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;

    // �������˲������¹�ת�Ƕ� X
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

}


