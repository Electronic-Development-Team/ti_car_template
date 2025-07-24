#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include "clock.h"



#define SPEED_RECORD_NUM 20
typedef struct
{
	uint8_t left_dir;
	uint8_t right_dir;
	uint16_t left_count;
	uint16_t right_count;
	uint16_t left_lastcount;
	uint16_t right_lastcount;
	float left_speed;
	float right_speed;	
  float left_speed_Record[SPEED_RECORD_NUM];
	float right_speed_Record[SPEED_RECORD_NUM];
} motor_encoder;

typedef struct {
  float Kp;           
  float Ki;           
  float Kd;           
  float integral;     
  float prev_error;  
  float output;
	float target_speed;
  float max_output;   
  float min_output; 
} PID_Controller;
extern motor_encoder encoder;
extern PID_Controller pidL, pidR;
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float min_out, float max_out);
void PID_Update(PID_Controller* pid, float error);
void go(int left,int right);
void left_go(int mode);//0:ahead 1:back 2:stop
void right_go(int mode);//0:ahead 1:back 2:stop

void encoder_init(motor_encoder *encoder);

#endif 