/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled_hardware_i2c.h"
#include "icm_spi.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>


volatile ICM_t icm_data;


void uart0_send_char(char ch);
void uart0_send_string(char* str);
void speed_uartsend(int VL, int VR);
void simple_motor_test(void);


int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
	  OLED_Init();
	  icm_init();
	  NVIC_ClearPendingIRQ(PWM_MOTOR_INST_INT_IRQN);
	  NVIC_EnableIRQ(PWM_MOTOR_INST_INT_IRQN);
	
		NVIC_EnableIRQ(ENCODER1A_INST_INT_IRQN);
		DL_TimerA_startCounter(ENCODER1A_INST);
	  NVIC_EnableIRQ(ENCODER2A_INST_INT_IRQN);
	  DL_TimerG_startCounter(ENCODER2A_INST);
	  PID_Init(&pidL, 0.15, 0.1, 2, 0, 1000);
    PID_Init(&pidR, 0.15, 0.1, 2, 0, 1000);
		//�����ʱ���жϱ�־
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //ʹ�ܶ�ʱ���ж�
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
		encoder_init(&encoder);
	//DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_AIN1_PIN );
	  pidL.target_speed=20;
		pidR.target_speed=20;
	//left_go(1);
	//right_go(1);
    while (1) 
		{
			// �򵥵ĵ������
			simple_motor_test();
			
			// ��ѡ����ʾ���������ݵ�OLED
			ICM_ReadSensors(&icm_data);
			
			if(icm_data.KalmanAngleX <0)
			{
				OLED_ShowChar(0,0,'-',16);
				OLED_ShowNum(8,0,(uint32_t)(-1*icm_data.KalmanAngleX),8,16);
			}
			else
			{
				OLED_ShowChar(0,0,'+',16);
				OLED_ShowNum(8,0,(uint32_t)icm_data.KalmanAngleX,8,16);
			}
			
			// ��ʾ�������ٶ�
			OLED_ShowNum(16,4,(uint32_t)encoder.left_speed,8,16);
			OLED_ShowNum(16,6,(uint32_t)encoder.right_speed,8,16);
			
			// С��ʱ����������ѭ��
			DL_Common_delayCycles(160000); // Լ2ms��ʱ (����80MHzʱ��)
    }
}


void uart0_send_char(char ch)
{
    //������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
    while( DL_UART_isBusy(UART_0_INST) == true );
    //���͵����ַ�
    DL_UART_Main_transmitData(UART_0_INST, ch);
}
//���ڷ����ַ���
void uart0_send_string(char* str)
{
    //��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
    while(*str!=0&&str!=0)
    {
        //�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
        uart0_send_char(*str++);
    }
}
void speed_uartsend(int VL, int VR)
{
    char txBuf[20];
    int len;
    
    // ��ʽ�����ݵ��ַ���
    len = sprintf(txBuf, "%d,%d\r\n", VL, VR);
    
    // ͨ��UART��������
    uart0_send_string(txBuf);
}

// �򵥵ĵ�����Ժ���
void simple_motor_test(void)
{
    static uint32_t test_counter = 0;
    static uint8_t test_state = 0;
    
    test_counter++;
    
    // ÿ1000��ѭ���л�һ��״̬��Լ1�룬ȡ������ѭ��Ƶ�ʣ�
    if (test_counter >= 1000) {
        test_counter = 0;
        test_state++;
        
        switch (test_state % 6) {
            case 0:
                // ֹͣ
                go(0, 0);
                uart0_send_string("Motors: STOP\r\n");
                break;
                
            case 1:
                // ���������ת������
                go(300, 300);
                uart0_send_string("Motors: Forward Low Speed\r\n");
                break;
                
            case 2:
                // ���������ת������
                go(500, 500);
                uart0_send_string("Motors: Forward Medium Speed\r\n");
                break;
                
            case 3:
                // ֹͣ
                go(0, 0);
                uart0_send_string("Motors: STOP\r\n");
                break;
                
            case 4:
                // ��ת - ������ת���ҵ����ת
                go(-300, 300);
                uart0_send_string("Motors: Turn Left\r\n");
                break;
                
            case 5:
                // ��ת - ������ת���ҵ����ת
                go(300, -300);
                uart0_send_string("Motors: Turn Right\r\n");
                break;
        }
    }
}

