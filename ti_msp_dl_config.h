/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG0
#define PWM_MOTOR_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_24
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM54)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM54_PF_TIMG0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for ENCODER1A */
#define ENCODER1A_INST                                                   (TIMA0)
#define ENCODER1A_INST_IRQHandler                               TIMA0_IRQHandler
#define ENCODER1A_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define ENCODER1A_INST_LOAD_VALUE                                        (9999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER1A_C0_PORT                                             GPIOB
#define GPIO_ENCODER1A_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_ENCODER1A_C0_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_ENCODER1A_C0_IOMUX_FUNC                 IOMUX_PINCM31_PF_TIMA0_CCP0

/* Defines for ENCODER2A */
#define ENCODER2A_INST                                                   (TIMG7)
#define ENCODER2A_INST_IRQHandler                               TIMG7_IRQHandler
#define ENCODER2A_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define ENCODER2A_INST_LOAD_VALUE                                        (9999U)
/* GPIO defines for channel 0 */
#define GPIO_ENCODER2A_C0_PORT                                             GPIOB
#define GPIO_ENCODER2A_C0_PIN                                     DL_GPIO_PIN_15
#define GPIO_ENCODER2A_C0_IOMUX                                  (IOMUX_PINCM32)
#define GPIO_ENCODER2A_C0_IOMUX_FUNC                 IOMUX_PINCM32_PF_TIMG7_CCP0





/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            4000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_0_FBRD_4_MHZ_9600_BAUD                                          (3)




/* Defines for SPI */
#define SPI_INST                                                           SPI1
#define SPI_INST_IRQHandler                                     SPI1_IRQHandler
#define SPI_INST_INT_IRQN                                         SPI1_INT_IRQn
#define GPIO_SPI_PICO_PORT                                                GPIOB
#define GPIO_SPI_PICO_PIN                                         DL_GPIO_PIN_8
#define GPIO_SPI_IOMUX_PICO                                     (IOMUX_PINCM25)
#define GPIO_SPI_IOMUX_PICO_FUNC                     IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_POCI_PORT                                                GPIOB
#define GPIO_SPI_POCI_PIN                                         DL_GPIO_PIN_7
#define GPIO_SPI_IOMUX_POCI                                     (IOMUX_PINCM24)
#define GPIO_SPI_IOMUX_POCI_FUNC                     IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI */
#define GPIO_SPI_SCLK_PORT                                                GPIOB
#define GPIO_SPI_SCLK_PIN                                         DL_GPIO_PIN_9
#define GPIO_SPI_IOMUX_SCLK                                     (IOMUX_PINCM26)
#define GPIO_SPI_IOMUX_SCLK_FUNC                     IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group L */
#define L_PORT                                                           (GPIOB)

/* Defines for LED1: GPIOB.22 with pinCMx 50 on package pin 21 */
#define L_LED1_PIN                                              (DL_GPIO_PIN_22)
#define L_LED1_IOMUX                                             (IOMUX_PINCM50)
/* Port definition for Pin Group CS */
#define CS_PORT                                                          (GPIOB)

/* Defines for PIN: GPIOB.6 with pinCMx 23 on package pin 58 */
#define CS_PIN_PIN                                               (DL_GPIO_PIN_6)
#define CS_PIN_IOMUX                                             (IOMUX_PINCM23)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for PIN_SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_OLED_PIN_SDA_PIN                                    (DL_GPIO_PIN_0)
#define GPIO_OLED_PIN_SDA_IOMUX                                   (IOMUX_PINCM1)
/* Defines for PIN_SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GPIO_OLED_PIN_SCL_PIN                                    (DL_GPIO_PIN_1)
#define GPIO_OLED_PIN_SCL_IOMUX                                   (IOMUX_PINCM2)
/* Port definition for Pin Group MOTOR_CONTROL */
#define MOTOR_CONTROL_PORT                                               (GPIOA)

/* Defines for AIN1: GPIOA.22 with pinCMx 47 on package pin 18 */
#define MOTOR_CONTROL_AIN1_PIN                                  (DL_GPIO_PIN_22)
#define MOTOR_CONTROL_AIN1_IOMUX                                 (IOMUX_PINCM47)
/* Defines for AIN2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define MOTOR_CONTROL_AIN2_PIN                                  (DL_GPIO_PIN_15)
#define MOTOR_CONTROL_AIN2_IOMUX                                 (IOMUX_PINCM37)
/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define MOTOR_CONTROL_BIN1_PIN                                  (DL_GPIO_PIN_16)
#define MOTOR_CONTROL_BIN1_IOMUX                                 (IOMUX_PINCM38)
/* Defines for BIN2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define MOTOR_CONTROL_BIN2_PIN                                  (DL_GPIO_PIN_17)
#define MOTOR_CONTROL_BIN2_IOMUX                                 (IOMUX_PINCM39)
/* Port definition for Pin Group MOTOR_ENCODER */
#define MOTOR_ENCODER_PORT                                               (GPIOB)

/* Defines for E1B: GPIOB.17 with pinCMx 43 on package pin 14 */
#define MOTOR_ENCODER_E1B_PIN                                   (DL_GPIO_PIN_17)
#define MOTOR_ENCODER_E1B_IOMUX                                  (IOMUX_PINCM43)
/* Defines for E2B: GPIOB.18 with pinCMx 44 on package pin 15 */
#define MOTOR_ENCODER_E2B_PIN                                   (DL_GPIO_PIN_18)
#define MOTOR_ENCODER_E2B_IOMUX                                  (IOMUX_PINCM44)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_ENCODER1A_init(void);
void SYSCFG_DL_ENCODER2A_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
