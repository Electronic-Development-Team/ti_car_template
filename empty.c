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
		//清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
		encoder_init(&encoder);
	//DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_AIN1_PIN );
	  pidL.target_speed=20;
		pidR.target_speed=20;
	//left_go(1);
	//right_go(1);
    while (1) 
		{
			// 简单的电机测试
			simple_motor_test();
			
			// 可选：显示传感器数据到OLED
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
			
			// 显示编码器速度
			OLED_ShowNum(16,4,(uint32_t)encoder.left_speed,8,16);
			OLED_ShowNum(16,6,(uint32_t)encoder.right_speed,8,16);
			
			// 小延时，避免过快的循环
			DL_Common_delayCycles(160000); // 约2ms延时 (假设80MHz时钟)
    }
}


void uart0_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_0_INST, ch);
}
//串口发送字符串
void uart0_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(*str++);
    }
}
void speed_uartsend(int VL, int VR)
{
    char txBuf[20];
    int len;
    
    // 格式化数据到字符串
    len = sprintf(txBuf, "%d,%d\r\n", VL, VR);
    
    // 通过UART发送数据
    uart0_send_string(txBuf);
}

// 简单的电机测试函数
void simple_motor_test(void)
{
    static uint32_t test_counter = 0;
    static uint8_t test_state = 0;
    
    test_counter++;
    
    // 每1000次循环切换一次状态（约1秒，取决于主循环频率）
    if (test_counter >= 1000) {
        test_counter = 0;
        test_state++;
        
        switch (test_state % 6) {
            case 0:
                // 停止
                go(0, 0);
                uart0_send_string("Motors: STOP\r\n");
                break;
                
            case 1:
                // 两个电机正转，低速
                go(300, 300);
                uart0_send_string("Motors: Forward Low Speed\r\n");
                break;
                
            case 2:
                // 两个电机正转，中速
                go(500, 500);
                uart0_send_string("Motors: Forward Medium Speed\r\n");
                break;
                
            case 3:
                // 停止
                go(0, 0);
                uart0_send_string("Motors: STOP\r\n");
                break;
                
            case 4:
                // 左转 - 左电机反转，右电机正转
                go(-300, 300);
                uart0_send_string("Motors: Turn Left\r\n");
                break;
                
            case 5:
                // 右转 - 左电机正转，右电机反转
                go(300, -300);
                uart0_send_string("Motors: Turn Right\r\n");
                break;
        }
    }
}

