#ifndef _CLOCK_H_
#define _CLOCK_H_

extern volatile unsigned long tick_ms;
extern volatile uint32_t start_time;
int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void SysTick_Init(void);

#endif  /* #ifndef _CLOCK_H_ */