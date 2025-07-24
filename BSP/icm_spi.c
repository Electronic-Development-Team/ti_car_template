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

        //发送数据
        DL_SPI_transmitData8(SPI_INST,dat);
        //等待SPI总线空闲
        while(DL_SPI_isBusy(SPI_INST));
        //接收数据
        data = DL_SPI_receiveData8(SPI_INST);
        //等待SPI总线空闲
        while(DL_SPI_isBusy(SPI_INST));

        return data;
}


uint8_t spi_read_byte(uint8_t addr)
{
	
	uint8_t temp = 0;	
	SPI_CS(0);
	spi_read_write_byte(addr|0x80);//发送读取ID命令
	temp=spi_read_write_byte(0xFF);
	SPI_CS(1);
	return temp;
	
}

uint8_t spi_write_byte(uint8_t addr,uint8_t data)
{
	
	uint8_t temp = 0;	
	SPI_CS(0);
	spi_read_write_byte(addr&0x7F);//发送读取ID命令
	temp=spi_read_write_byte(data);
	SPI_CS(1);
	return temp;
	
}
//读取芯片ID
//返回值如下:
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128
//读取设备ID


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
	  //data = spi_read_byte(ICM_GYRO_CONFIG0);//加速度量程寄存器

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
	
	DataH = spi_read_byte(ICM_ACCEL_DATA_Z1);		//读取加速度计Z轴的高8位数据
	DataL = spi_read_byte(ICM_ACCEL_DATA_Z0);		//读取加速度计Z轴的低8位数据
	DataStruct->Accel_Z_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = spi_read_byte(ICM_GYRO_DATA_X1);		//读取陀螺仪X轴的高8位数据
	DataL = spi_read_byte(ICM_GYRO_DATA_X0);		//读取陀螺仪X轴的低8位数据
	DataStruct->Gyro_X_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = spi_read_byte(ICM_GYRO_DATA_Y1);		//读取陀螺仪Y轴的高8位数据
	DataL = spi_read_byte(ICM_GYRO_DATA_Y0);		//读取陀螺仪Y轴的低8位数据
	DataStruct->Gyro_Y_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = spi_read_byte(ICM_GYRO_DATA_Z1);		//读取陀螺仪Z轴的高8位数据
	DataL = spi_read_byte(ICM_GYRO_DATA_Z0);		//读取陀螺仪Z轴的低8位数据
	DataStruct->Gyro_Z_RAW = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = spi_read_byte(ICM_TEMP_DATA1);		//读取陀螺仪Z轴的高8位数据
	DataL = spi_read_byte(ICM_TEMP_DATA0);		//读取陀螺仪Z轴的低8位数据
	DataStruct->Temperature = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	
}
float ICM_ConvertAccel(int16_t value, uint8_t range) 
{
    float factor; // 用于存储转换因子
    // 根据加速度计的量程选择合适的转换因子
    switch (range) {
        case 0x60: factor = 16384.0; break; // ±2g，转换因子为16384
        case 0x40: factor = 8192.0; break;  // ±4g，转换因子为8192
        case 0x20: factor = 4096.0; break;  // ±8g，转换因子为4096
        case 0x00: factor = 2048.0; break;  // ±16g，转换因子为2048
        default: factor = 16384.0; break;    // 默认量程范围为±2g
    }
    // 将原始加速度值转换为g单位
    return value / factor;
}
 
float ICM_ConvertGyro(int16_t value, uint8_t range) 
{
    float factor; // 用于存储转换因子
    // 根据陀螺仪的量程选择合适的转换因子
    switch (range) {
        case 0x60: factor = 131.0; break;  // ±250°/s，转换因子为131
        case 0x40: factor = 65.5; break;   // ±500°/s，转换因子为65.5
        case 0x20: factor = 32.8; break;   // ±1000°/s，转换因子为32.8
        case 0x00: factor = 16.4; break;   // ±2000°/s，转换因子为16.4
        default: factor = 131.0; break;     // 默认量程范围为±250°/s
    }
    // 将原始陀螺仪值转换为°/s单位
    return value / factor;
}

//温度数据转化
float ICM_ConvertTemp(int16_t value) 
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

void ICM_ReadSensors(ICM_t *DataStruct) 
{
	unsigned long count;
	ICM_ReadData(DataStruct);// 读取加速度计和陀螺仪的原始数据

    // 将加速度计的原始数据转换为浮点数形式（单位：g）
    DataStruct->Ax = ICM_ConvertAccel(DataStruct->Accel_X_RAW, ICM_ACCEL_RANGE); // ±2g量程
    DataStruct->Ay = ICM_ConvertAccel(DataStruct->Accel_Y_RAW, ICM_ACCEL_RANGE); // ±2g量程
    DataStruct->Az = ICM_ConvertAccel(DataStruct->Accel_Z_RAW, ICM_ACCEL_RANGE); // ±2g量程
    // 将陀螺仪的原始数据转换为浮点数形式（单位：°/s）
    DataStruct->Gx = ICM_ConvertGyro(DataStruct->Gyro_X_RAW, ICM_GYRO_RANGE)*AtR; // ±250°/s量程
    DataStruct->Gy = ICM_ConvertGyro(DataStruct->Gyro_Y_RAW, ICM_GYRO_RANGE)*AtR; // ±250°/s量程
    DataStruct->Gz = ICM_ConvertGyro(DataStruct->Gyro_Z_RAW, ICM_GYRO_RANGE)*AtR; // ±250°/s量程
		//温度
		DataStruct->Temperature = ICM_ConvertTemp(DataStruct->Temperature);
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

////发送写使能
//void W25Q128_write_enable(void)
//{
//    //拉低CS端为低电平
//    SPI_CS(0);
//    //发送指令06h
//    spi_read_write_byte(0x06);
//    //拉高CS端为高电平
//    SPI_CS(1);
//}

//void W25Q128_wait_busy(void)
//{
//	unsigned char byte = 0;
//	do
//	 {
//			//拉低CS端为低电平
//			SPI_CS(0);
//			//发送指令05h
//			spi_read_write_byte(0x05);
//			//接收状态寄存器值
//			byte = spi_read_write_byte(0Xff);
//			//恢复CS端为高电平
//			SPI_CS(1);
//	 //判断BUSY位是否为1 如果为1说明在忙，重新读写BUSY位直到为0
//	 }while( ( byte & 0x01 ) == 1 );
//}

///**********************************************************
// * 函 数 名 称：W25Q128_erase_sector
// * 函 数 功 能：擦除一个扇区
// * 传 入 参 数：addr=擦除的扇区号
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：addr=擦除的扇区号，范围=0~15
//**********************************************************/
//void W25Q128_erase_sector(uint32_t addr)
//{
//	//计算扇区号，一个扇区4KB=4096
//	addr *= 4096;
//	W25Q128_write_enable();  //写使能
//	W25Q128_wait_busy();     //判断忙，如果忙则一直等待
//	//拉低CS端为低电平
//	SPI_CS(0);
//	//发送指令20h
//	spi_read_write_byte(0x20);
//	//发送24位扇区地址的高8位
//	spi_read_write_byte((uint8_t)((addr)>>16));
//	//发送24位扇区地址的中8位
//	spi_read_write_byte((uint8_t)((addr)>>8));
//	//发送24位扇区地址的低8位
//	spi_read_write_byte((uint8_t)addr);
//	//恢复CS端为高电平
//	SPI_CS(1);
//	//等待擦除完成
//	W25Q128_wait_busy();
//}

///**********************************************************
// * 函 数 名 称：W25Q128_write
// * 函 数 功 能：写数据到W25Q128进行保存
// * 传 入 参 数：buffer=写入的数据内容        addr=写入地址        numbyte=写入数据的长度
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//**********************************************************/
//void W25Q128_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
//{
//    unsigned int i = 0;
//    //擦除扇区数据
//    W25Q128_erase_sector(addr/4096);
//    //写使能
//    W25Q128_write_enable();
//    //忙检测
//    W25Q128_wait_busy();
//    //写入数据
//    //拉低CS端为低电平
//    SPI_CS(0);
//    //发送指令02h
//    spi_read_write_byte(0x02);
//    //发送写入的24位地址中的高8位
//    spi_read_write_byte((uint8_t)((addr)>>16));
//    //发送写入的24位地址中的中8位
//    spi_read_write_byte((uint8_t)((addr)>>8));
//    //发送写入的24位地址中的低8位
//    spi_read_write_byte((uint8_t)addr);
//    //根据写入的字节长度连续写入数据buffer
//    for(i=0;i<numbyte;i++)
//    {
//        spi_read_write_byte(buffer[i]);
//    }
//    //恢复CS端为高电平
//    SPI_CS(0);
//    //忙检测
//    W25Q128_wait_busy();
//}

///**********************************************************
// * 函 数 名 称：W25Q128_read
// * 函 数 功 能：读取W25Q128的数据
// * 传 入 参 数：buffer=读出数据的保存地址  read_addr=读取地址   read_length=读去长度
// * 函 数 返 回：无
// * 作       者：LC
// * 备       注：无
//**********************************************************/
//void W25Q128_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)
//{
//	uint16_t i;
//	//拉低CS端为低电平
//	SPI_CS(0);
//	//发送指令03h
//	spi_read_write_byte(0x03);
//	//发送24位读取数据地址的高8位
//	spi_read_write_byte((uint8_t)((read_addr)>>16));
//	//发送24位读取数据地址的中8位
//	spi_read_write_byte((uint8_t)((read_addr)>>8));
//	//发送24位读取数据地址的低8位
//	spi_read_write_byte((uint8_t)read_addr);
//	//根据读取长度读取出地址保存到buffer中
//	for(i=0;i<read_length;i++)
//	{
//		buffer[i]= spi_read_write_byte(0XFF);
//	}
//	//恢复CS端为高电平
//	SPI_CS(1);
//}