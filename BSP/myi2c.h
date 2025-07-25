#ifndef __MYI2C_H
#define __MYI2C_H
#include "stdint.h"
#include "ti_msp_dl_config.h"
#include "clock.h"
void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_R_SDA(void);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);
void MyI2C_W_SCL(uint8_t BitValue);
void MyI2C_W_SDA(uint8_t BitValue);
void Delay_us(uint16_t us);
 
#endif
