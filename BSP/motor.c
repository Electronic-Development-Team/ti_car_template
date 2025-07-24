#include "motor.h"

motor_encoder encoder;
PID_Controller pidL, pidR;
float error;
void go(int left,int right)
{
	if(left>=0)
	{
		left_go(1);
		encoder.left_dir=1;
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,left,GPIO_PWM_MOTOR_C0_IDX);
	}
	else
	{
		encoder.left_dir=0;
		left_go(0);
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,-1*left,GPIO_PWM_MOTOR_C0_IDX);
	}
	if(right>=0)
	{
		right_go(1);
		encoder.right_dir=1;
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,right,GPIO_PWM_MOTOR_C1_IDX);
	}
	else
	{
		right_go(0);
		encoder.right_dir=0;
		DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,-1*right,GPIO_PWM_MOTOR_C1_IDX);
	}
}

void left_go(int mode)//0:ahead 1:back 2:stop
{
	switch(mode)
	{
		case 0:
			 DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_AIN2_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_AIN1_PIN );
			 break;
		case 1:
			 DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_AIN1_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_AIN2_PIN );
       break;
		case 2:
			 DL_GPIO_clearPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_AIN1_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_AIN2_PIN );
       break;
		default :
			break;
		
	}
}

void right_go(int mode)//0:ahead 1:back 2:stop
{
	switch(mode)
	{
		case 0:
			 DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_BIN2_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_BIN1_PIN );
			break;
		case 1:
			 DL_GPIO_setPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_BIN1_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_BIN2_PIN );
			break;
		case 2:
			 DL_GPIO_clearPins(MOTOR_CONTROL_PORT,MOTOR_CONTROL_BIN1_PIN );
		   DL_GPIO_clearPins(MOTOR_CONTROL_PORT, MOTOR_CONTROL_BIN2_PIN );
			break;
		default :
			break;
		
	}
}

void encoder_init(motor_encoder *encoder)
{
	encoder->left_dir=0;
	encoder->right_dir=0;
	encoder->left_count=0;
	encoder->right_count=0;
	encoder->left_lastcount=0;
	encoder->right_lastcount=0;
	encoder->left_speed=0;
	encoder->right_speed=0;	
}

void PWM_MOTOR_INST_IRQHandler(void)
{
    switch( DL_TimerG_getPendingInterrupt(PWM_MOTOR_INST) )
    {
        case DL_TIMER_IIDX_LOAD:
						
            
            break;
        default:
            break;
    }
		NVIC_ClearPendingIRQ(PWM_MOTOR_INST_INT_IRQN); 
}
void ENCODER1A_INST_IRQHandler(void)
{
    switch( DL_TimerA_getPendingInterrupt(ENCODER1A_INST) )
    {
        case DL_TIMERA_IIDX_CC0_DN:
						//encoder.left_dir=DL_GPIO_readPins(MOTOR_ENCODER_PORT, MOTOR_ENCODER_E1B_PIN);
            encoder.left_count =
        encoder.left_dir ? ( encoder.left_count + 1) : ( encoder.left_count- 1);//ͨ���ж���ת����������countnum���ӻ��Ǽ���

            break;
        default:
            break;
    }
		NVIC_ClearPendingIRQ(PWM_MOTOR_INST_INT_IRQN); 
}
void ENCODER2A_INST_IRQHandler(void)
{
    switch( DL_TimerG_getPendingInterrupt(ENCODER2A_INST) )
    {
        case DL_TIMERG_IIDX_CC0_DN:
						//encoder.right_dir=DL_GPIO_readPins(MOTOR_ENCODER_PORT, MOTOR_ENCODER_E2B_PIN);
            encoder.right_count =
        encoder.right_dir ? ( encoder.right_count + 1) : ( encoder.right_count- 1);//ͨ���ж���ת����������countnum���ӻ��Ǽ���

            
            break;
        default:
            break;
    }
		NVIC_ClearPendingIRQ(PWM_MOTOR_INST_INT_IRQN); 
}

void Speed_Low_Filter(motor_encoder *encoder)
{
    float left_sum = 0.0f;
	  float right_sum = 0.0f;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)
    {
        encoder->left_speed_Record[i] = encoder->left_speed_Record[i-1];
        left_sum += encoder->left_speed_Record[i-1];
			  encoder->right_speed_Record[i] = encoder->right_speed_Record[i-1];
        right_sum += encoder->right_speed_Record[i-1];
    }
		
    encoder->left_speed_Record[0] = encoder->left_speed;
    left_sum += encoder->left_speed;
    encoder->left_speed= left_sum/SPEED_RECORD_NUM;
		
		encoder->right_speed_Record[0] = encoder->right_speed;
    right_sum += encoder->right_speed;
    encoder->right_speed= right_sum/SPEED_RECORD_NUM;
}

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->min_output = min_out;
    pid->max_output = max_out;
}

void PID_Update(PID_Controller* pid, float error)//pid更新
 {
			
    pid->integral += error ;
   
    float derivative = error - pid->prev_error;
    
    pid->output = pid->Kp * error + 
                 pid->Ki * pid->integral + 
                 pid->Kd * derivative;
    
    if(pid->output > pid->max_output) pid->output = pid->max_output;
    if(pid->output < pid->min_output) pid->output = pid->min_output;
    
    pid->prev_error = error;
}


void TIMER_0_INST_IRQHandler(void)
{

    switch( DL_TimerA_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMER_IIDX_LOAD:

            encoder.left_speed=(encoder.left_count-encoder.left_lastcount)*100.f/30.f;
						encoder.right_speed=(encoder.right_count-encoder.right_lastcount)*100.f/30.f;
						Speed_Low_Filter(&encoder);
				    encoder.left_lastcount=encoder.left_count;
						encoder.right_lastcount=encoder.right_count;
						error=pidL.target_speed-encoder.left_speed;
            PID_Update(&pidL,error);  
						error=pidR.target_speed-encoder.right_speed;
            PID_Update(&pidR,error);  
            break;

        default:
            break;
    }
		NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
}
