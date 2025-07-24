#include "icm_spi.h"



Kalman_t KalmanX = {
    .Q_angle = 0.05f,
    .Q_bias = 0.05f,
    .R_measure = 0.01f};

Kalman_t KalmanY = {
    .Q_angle = 0.05f,
    .Q_bias = 0.05f,
    .R_measure = 0.01f,
};

uint32_t timer;





uint8_t spi_read_write_byte(uint8_t dat)
{
        uint8_t data = 0;

        //��������
        DL_SPI_transmitData8(SPI_INST,dat);
        //�ȴ�SPI���߿���
        while(DL_SPI_isBusy(SPI_INST));
        //��������
        data = DL_SPI_receiveData8(SPI_INST);
        //�ȴ�SPI���߿���
        while(DL_SPI_isBusy(SPI_INST));

        return data;
}


uint8_t spi_read_byte(uint8_t addr)
{
	
	uint8_t temp = 0;	
	SPI_CS(0);
	spi_read_write_byte(addr|0x80);//���Ͷ�ȡID����
	temp=spi_read_write_byte(0xFF);
	SPI_CS(1);
	return temp;
	
}

uint8_t spi_write_byte(uint8_t addr,uint8_t data)
{
	
	uint8_t temp = 0;	
	SPI_CS(0);
	spi_read_write_byte(addr&0x7F);//���Ͷ�ȡID����
	temp=spi_read_write_byte(data);
	SPI_CS(1);
	return temp;
	
}
//��ȡоƬID
//����ֵ����:
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128
//��ȡ�豸ID


uint8_t icm_readID(void)
{
		uint8_t temp = 0;
    temp=spi_read_byte(ICM_WHOAMI);
    return temp;
}



void icm_init(void)
{
		uint8_t data;
	  spi_write_byte(ICM_REG_BANK_SEL,0x00);
	  //mspm0_delay_ms(4);
    spi_write_byte(ICM_DEVICE_CONFIG,0x01);
	  //mspm0_delay_ms(4);
		//data=spi_read_byte(ICM_PWR_MGMT0);
		
	  mspm0_delay_ms(100);
	  //data = spi_read_byte(ICM_GYRO_CONFIG0);//���ٶ����̼Ĵ���

	  spi_write_byte(ICM_GYRO_CONFIG0,0x06);
	
    //data = spi_read_byte(ICM_ACCEL_CONFIG0);

	  spi_write_byte(ICM_ACCEL_CONFIG0,0x06);
	  spi_write_byte(ICM_PWR_MGMT0,0x0F);
}

void ICM_ReadData(ICM_t * DataStruct)
{
	uint8_t  DataH, DataL;
	uint8_t buf[12];
	DataH = spi_read_byte(ICM_ACCEL_DATA_X1);
	DataL = spi_read_byte(ICM_ACCEL_DATA_X0);
	DataStruct->Accel_X_RAW = (DataH << 8) | DataL;
	
	DataH = spi_read_byte(ICM_ACCEL_DATA_Y1);
	DataL = spi_read_byte(ICM_ACCEL_DATA_Y0);
	DataStruct->Accel_Y_RAW = (DataH << 8) | DataL;
	
	DataH = spi_read_byte(ICM_ACCEL_DATA_Z1);		//��ȡ���ٶȼ�Z��ĸ�8λ����
	DataL = spi_read_byte(ICM_ACCEL_DATA_Z0);		//��ȡ���ٶȼ�Z��ĵ�8λ����
	DataStruct->Accel_Z_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = spi_read_byte(ICM_GYRO_DATA_X1);		//��ȡ������X��ĸ�8λ����
	DataL = spi_read_byte(ICM_GYRO_DATA_X0);		//��ȡ������X��ĵ�8λ����
	DataStruct->Gyro_X_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = spi_read_byte(ICM_GYRO_DATA_Y1);		//��ȡ������Y��ĸ�8λ����
	DataL = spi_read_byte(ICM_GYRO_DATA_Y0);		//��ȡ������Y��ĵ�8λ����
	DataStruct->Gyro_Y_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = spi_read_byte(ICM_GYRO_DATA_Z1);		//��ȡ������Z��ĸ�8λ����
	DataL = spi_read_byte(ICM_GYRO_DATA_Z0);		//��ȡ������Z��ĵ�8λ����
	DataStruct->Gyro_Z_RAW = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	DataH = spi_read_byte(ICM_TEMP_DATA1);		//��ȡ������Z��ĸ�8λ����
	DataL = spi_read_byte(ICM_TEMP_DATA0);		//��ȡ������Z��ĵ�8λ����
	DataStruct->Temperature = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
	
	
}
float ICM_ConvertAccel(int16_t value, uint8_t range) 
{
    float factor; // ���ڴ洢ת������
    // ���ݼ��ٶȼƵ�����ѡ����ʵ�ת������
    switch (range) {
        case 0x60: factor = 16384.0; break; // ��2g��ת������Ϊ16384
        case 0x40: factor = 8192.0; break;  // ��4g��ת������Ϊ8192
        case 0x20: factor = 4096.0; break;  // ��8g��ת������Ϊ4096
        case 0x00: factor = 2048.0; break;  // ��16g��ת������Ϊ2048
        default: factor = 16384.0; break;    // Ĭ�����̷�ΧΪ��2g
    }
    // ��ԭʼ���ٶ�ֵת��Ϊg��λ
    return value / factor;
}
 
float ICM_ConvertGyro(int16_t value, uint8_t range) 
{
    float factor; // ���ڴ洢ת������
    // ���������ǵ�����ѡ����ʵ�ת������
    switch (range) {
        case 0x60: factor = 131.0; break;  // ��250��/s��ת������Ϊ131
        case 0x40: factor = 65.5; break;   // ��500��/s��ת������Ϊ65.5
        case 0x20: factor = 32.8; break;   // ��1000��/s��ת������Ϊ32.8
        case 0x00: factor = 16.4; break;   // ��2000��/s��ת������Ϊ16.4
        default: factor = 131.0; break;     // Ĭ�����̷�ΧΪ��250��/s
    }
    // ��ԭʼ������ֵת��Ϊ��/s��λ
    return value / factor;
}

//�¶�����ת��
float ICM_ConvertTemp(int16_t value) 
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

void ICM_ReadSensors(ICM_t *DataStruct) 
{
	unsigned long count;
	ICM_ReadData(DataStruct);// ��ȡ���ٶȼƺ������ǵ�ԭʼ����

    // �����ٶȼƵ�ԭʼ����ת��Ϊ��������ʽ����λ��g��
    DataStruct->Ax = ICM_ConvertAccel(DataStruct->Accel_X_RAW, ICM_ACCEL_RANGE); // ��2g����
    DataStruct->Ay = ICM_ConvertAccel(DataStruct->Accel_Y_RAW, ICM_ACCEL_RANGE); // ��2g����
    DataStruct->Az = ICM_ConvertAccel(DataStruct->Accel_Z_RAW, ICM_ACCEL_RANGE); // ��2g����
    // �������ǵ�ԭʼ����ת��Ϊ��������ʽ����λ����/s��
    DataStruct->Gx = ICM_ConvertGyro(DataStruct->Gyro_X_RAW, ICM_GYRO_RANGE)*AtR; // ��250��/s����
    DataStruct->Gy = ICM_ConvertGyro(DataStruct->Gyro_Y_RAW, ICM_GYRO_RANGE)*AtR; // ��250��/s����
    DataStruct->Gz = ICM_ConvertGyro(DataStruct->Gyro_Z_RAW, ICM_GYRO_RANGE)*AtR; // ��250��/s����
		//�¶�
		DataStruct->Temperature = ICM_ConvertTemp(DataStruct->Temperature);
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

////����дʹ��
//void W25Q128_write_enable(void)
//{
//    //����CS��Ϊ�͵�ƽ
//    SPI_CS(0);
//    //����ָ��06h
//    spi_read_write_byte(0x06);
//    //����CS��Ϊ�ߵ�ƽ
//    SPI_CS(1);
//}

//void W25Q128_wait_busy(void)
//{
//	unsigned char byte = 0;
//	do
//	 {
//			//����CS��Ϊ�͵�ƽ
//			SPI_CS(0);
//			//����ָ��05h
//			spi_read_write_byte(0x05);
//			//����״̬�Ĵ���ֵ
//			byte = spi_read_write_byte(0Xff);
//			//�ָ�CS��Ϊ�ߵ�ƽ
//			SPI_CS(1);
//	 //�ж�BUSYλ�Ƿ�Ϊ1 ���Ϊ1˵����æ�����¶�дBUSYλֱ��Ϊ0
//	 }while( ( byte & 0x01 ) == 1 );
//}

///**********************************************************
// * �� �� �� �ƣ�W25Q128_erase_sector
// * �� �� �� �ܣ�����һ������
// * �� �� �� ����addr=������������
// * �� �� �� �أ���
// * ��       �ߣ�LC
// * ��       ע��addr=�����������ţ���Χ=0~15
//**********************************************************/
//void W25Q128_erase_sector(uint32_t addr)
//{
//	//���������ţ�һ������4KB=4096
//	addr *= 4096;
//	W25Q128_write_enable();  //дʹ��
//	W25Q128_wait_busy();     //�ж�æ�����æ��һֱ�ȴ�
//	//����CS��Ϊ�͵�ƽ
//	SPI_CS(0);
//	//����ָ��20h
//	spi_read_write_byte(0x20);
//	//����24λ������ַ�ĸ�8λ
//	spi_read_write_byte((uint8_t)((addr)>>16));
//	//����24λ������ַ����8λ
//	spi_read_write_byte((uint8_t)((addr)>>8));
//	//����24λ������ַ�ĵ�8λ
//	spi_read_write_byte((uint8_t)addr);
//	//�ָ�CS��Ϊ�ߵ�ƽ
//	SPI_CS(1);
//	//�ȴ��������
//	W25Q128_wait_busy();
//}

///**********************************************************
// * �� �� �� �ƣ�W25Q128_write
// * �� �� �� �ܣ�д���ݵ�W25Q128���б���
// * �� �� �� ����buffer=д�����������        addr=д���ַ        numbyte=д�����ݵĳ���
// * �� �� �� �أ���
// * ��       �ߣ�LC
// * ��       ע����
//**********************************************************/
//void W25Q128_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
//{
//    unsigned int i = 0;
//    //������������
//    W25Q128_erase_sector(addr/4096);
//    //дʹ��
//    W25Q128_write_enable();
//    //æ���
//    W25Q128_wait_busy();
//    //д������
//    //����CS��Ϊ�͵�ƽ
//    SPI_CS(0);
//    //����ָ��02h
//    spi_read_write_byte(0x02);
//    //����д���24λ��ַ�еĸ�8λ
//    spi_read_write_byte((uint8_t)((addr)>>16));
//    //����д���24λ��ַ�е���8λ
//    spi_read_write_byte((uint8_t)((addr)>>8));
//    //����д���24λ��ַ�еĵ�8λ
//    spi_read_write_byte((uint8_t)addr);
//    //����д����ֽڳ�������д������buffer
//    for(i=0;i<numbyte;i++)
//    {
//        spi_read_write_byte(buffer[i]);
//    }
//    //�ָ�CS��Ϊ�ߵ�ƽ
//    SPI_CS(0);
//    //æ���
//    W25Q128_wait_busy();
//}

///**********************************************************
// * �� �� �� �ƣ�W25Q128_read
// * �� �� �� �ܣ���ȡW25Q128������
// * �� �� �� ����buffer=�������ݵı����ַ  read_addr=��ȡ��ַ   read_length=��ȥ����
// * �� �� �� �أ���
// * ��       �ߣ�LC
// * ��       ע����
//**********************************************************/
//void W25Q128_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)
//{
//	uint16_t i;
//	//����CS��Ϊ�͵�ƽ
//	SPI_CS(0);
//	//����ָ��03h
//	spi_read_write_byte(0x03);
//	//����24λ��ȡ���ݵ�ַ�ĸ�8λ
//	spi_read_write_byte((uint8_t)((read_addr)>>16));
//	//����24λ��ȡ���ݵ�ַ����8λ
//	spi_read_write_byte((uint8_t)((read_addr)>>8));
//	//����24λ��ȡ���ݵ�ַ�ĵ�8λ
//	spi_read_write_byte((uint8_t)read_addr);
//	//���ݶ�ȡ���ȶ�ȡ����ַ���浽buffer��
//	for(i=0;i<read_length;i++)
//	{
//		buffer[i]= spi_read_write_byte(0XFF);
//	}
//	//�ָ�CS��Ϊ�ߵ�ƽ
//	SPI_CS(1);
//}