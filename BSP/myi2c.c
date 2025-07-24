#include "ti_msp_dl_config.h"
 
#include "ti/driverlib/dl_gpio.h"
#include "myi2c.h"
//��SDA���ţ������
void SDA_OUT(void)   
{
    DL_GPIO_initDigitalOutput(GPIO_OLED_PIN_SDA_IOMUX  );     
	  DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN);	   
    DL_GPIO_enableOutput(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN); 
}
//�ر�SDA���ţ����룩
void SDA_IN(void)
{
 
    DL_GPIO_initDigitalInputFeatures(GPIO_OLED_PIN_SDA_IOMUX  ,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
 
 
}
 
void Delay_us(uint16_t us)
{
    while(us--)
    delay_cycles(CPUCLK_FREQ/1000000);
}//CPUCLK_FREQΪʱ��Ƶ�ʣ����Ը������õĸı���ı�
/*�������ò�*/
 
/**
  * ��    ����I2CдSCL���ŵ�ƽ
  * ��    ����BitValue Э��㴫��ĵ�ǰ��Ҫд��SCL�ĵ�ƽ����Χ0~1
  * �� �� ֵ����
  * ע������˺�����Ҫ�û�ʵ�����ݣ���BitValueΪ0ʱ����Ҫ��SCLΪ�͵�ƽ����BitValueΪ1ʱ����Ҫ��SCLΪ�ߵ�ƽ
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
    if(BitValue)
        DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SCL_PIN);
    else
        DL_GPIO_clearPins(GPIO_OLED_PORT,GPIO_OLED_PIN_SCL_PIN);
	Delay_us(8);	//��ʱ8us����ֹʱ��Ƶ�ʳ���Ҫ��
}
 
/**
  * ��    ����I2CдSDA���ŵ�ƽ
  * ��    ����BitValue Э��㴫��ĵ�ǰ��Ҫд��SDA�ĵ�ƽ����Χ0~0xFF
  * �� �� ֵ����
  * ע������˺�����Ҫ�û�ʵ�����ݣ���BitValueΪ0ʱ����Ҫ��SDAΪ�͵�ƽ����BitValue��0ʱ����Ҫ��SDAΪ�ߵ�ƽ
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
    SDA_OUT();
    if(BitValue)
        DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN);
    else
        DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN);
	Delay_us(8);					//��ʱ8us����ֹʱ��Ƶ�ʳ���Ҫ��
}
 
/**
  * ��    ����I2C��SDA���ŵ�ƽ
  * ��    ������
  * �� �� ֵ��Э�����Ҫ�õ��ĵ�ǰSDA�ĵ�ƽ����Χ0~1
  * ע������˺�����Ҫ�û�ʵ�����ݣ���ǰSDAΪ�͵�ƽʱ������0����ǰSDAΪ�ߵ�ƽʱ������1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t b;
    uint32_t BitValue;
    SDA_IN();
	BitValue = DL_GPIO_readPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN);		//��ȡSDA��ƽ
    {
        if(BitValue)   b=1;
        else           b=0;
    }
	Delay_us(8);		//��ʱ8us����ֹʱ��Ƶ�ʳ���Ҫ��
	return b;	        //����SDA��ƽ
}
 
/**
  * ��    ����I2C��ʼ��
  * ��    ������
  * �� �� ֵ����
  * ע������˺�����Ҫ�û�ʵ�����ݣ�ʵ��SCL��SDA���ŵĳ�ʼ��
  */
void MyI2C_Init(void)
{
    SYSCFG_DL_GPIO_init();
	/*����Ĭ�ϵ�ƽ*/
	DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_PIN_SDA_PIN |
		GPIO_OLED_PIN_SCL_PIN);//����PA8��PA9���ų�ʼ����Ĭ��Ϊ�ߵ�ƽ���ͷ�����״̬��
}
 
/*Э���*/
 
/**
  * ��    ����I2C��ʼ
  * ��    ������
  * �� �� ֵ����
  */
void MyI2C_Start(void)
{
    SDA_OUT();
	MyI2C_W_SDA(1);				//�ͷ�SDA��ȷ��SDAΪ�ߵ�ƽ
	MyI2C_W_SCL(1);				//�ͷ�SCL��ȷ��SCLΪ�ߵ�ƽ
	MyI2C_W_SDA(0);				//��SCL�ߵ�ƽ�ڼ䣬����SDA��������ʼ�ź�
	MyI2C_W_SCL(0);				//��ʼ������SCL��Ϊ��ռ�����ߣ���������ʱ���ƴ��
}
 
/**
  * ��    ����I2C��ֹ
  * ��    ������
  * �� �� ֵ����
  */
void MyI2C_Stop(void)
{
    SDA_OUT();
	MyI2C_W_SDA(0);							//����SDA��ȷ��SDAΪ�͵�ƽ
	MyI2C_W_SCL(1);							//�ͷ�SCL��ʹSCL���ָߵ�ƽ
	MyI2C_W_SDA(1);							//��SCL�ߵ�ƽ�ڼ䣬�ͷ�SDA��������ֹ�ź�
}
 
/**
  * ��    ����I2C����һ���ֽ�
  * ��    ����Byte Ҫ���͵�һ���ֽ����ݣ���Χ��0x00~0xFF
  * �� �� ֵ����
  */
void MyI2C_SendByte(uint8_t Byte)
{
    SDA_OUT();
	uint8_t i;
	for (i = 0; i < 8; i ++)				//ѭ��8�Σ��������η������ݵ�ÿһλ
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));	//ʹ������ķ�ʽȡ��Byte��ָ��һλ���ݲ�д�뵽SDA��
		MyI2C_W_SCL(1);						//�ͷ�SCL���ӻ���SCL�ߵ�ƽ�ڼ��ȡSDA
		MyI2C_W_SCL(0);						//����SCL��������ʼ������һλ����
	}
}
 
/**
  * ��    ����I2C����һ���ֽ�
  * ��    ������
  * �� �� ֵ�����յ���һ���ֽ����ݣ���Χ��0x00~0xFF
  */
uint8_t MyI2C_ReceiveByte(void)
{
    SDA_OUT();
	uint8_t i, Byte = 0x00;	//������յ����ݣ�������ֵ0x00
	MyI2C_W_SDA(1);			//����ǰ��������ȷ���ͷ�SDA��������Ŵӻ������ݷ���
	for (i = 0; i < 8; i ++)	//ѭ��8�Σ��������ν������ݵ�ÿһλ
	{
        SDA_IN();
		MyI2C_W_SCL(1);						//�ͷ�SCL����������SCL�ߵ�ƽ�ڼ��ȡSDA
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}	//��ȡSDA���ݣ����洢��Byte����
	//��SDAΪ1ʱ���ñ���ָ��λΪ1����SDAΪ0ʱ����������ָ��λΪĬ�ϵĳ�ֵ0
		MyI2C_W_SCL(0);						//����SCL���ӻ���SCL�͵�ƽ�ڼ�д��SDA
	}
	return Byte;							//���ؽ��յ���һ���ֽ�����
}
 
/**
  * ��    ����I2C����Ӧ��λ
  * ��    ����Byte Ҫ���͵�Ӧ��λ����Χ��0~1��0��ʾӦ��1��ʾ��Ӧ��
  * �� �� ֵ����
  */
void MyI2C_SendAck(uint8_t AckBit)
{
    SDA_OUT();
	MyI2C_W_SDA(AckBit);					//������Ӧ��λ���ݷŵ�SDA��
	MyI2C_W_SCL(1);							//�ͷ�SCL���ӻ���SCL�ߵ�ƽ�ڼ䣬��ȡӦ��λ
	MyI2C_W_SCL(0);							//����SCL����ʼ��һ��ʱ��ģ��
}
 
/**
  * ��    ����I2C����Ӧ��λ
  * ��    ������
  * �� �� ֵ�����յ���Ӧ��λ����Χ��0~1��0��ʾӦ��1��ʾ��Ӧ��
  */
uint8_t MyI2C_ReceiveAck(void)
{
    SDA_OUT();
	uint8_t AckBit;							//����Ӧ��λ����
	MyI2C_W_SDA(1);							//����ǰ��������ȷ���ͷ�SDA��������Ŵӻ������ݷ���
	MyI2C_W_SCL(1);							//�ͷ�SCL����������SCL�ߵ�ƽ�ڼ��ȡSDA
    SDA_IN();
	AckBit = MyI2C_R_SDA();					//��Ӧ��λ�洢��������
	MyI2C_W_SCL(0);							//����SCL����ʼ��һ��ʱ��ģ��
	return AckBit;							//���ض���Ӧ��λ����
}
