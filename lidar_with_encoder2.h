/**
 * lidar_with_encoder2.h
 * STM32F103C8T6
 * Dual MT6825 encoder + dual VB22A LiDAR + dual motor driver
 * Separate file from lidar_with_encoder.h/lidar_with_encoder.c
 *
 * PIN MAP (example):
 *   SPI1  PA5=SCK  PA6=MISO  PA7=MOSI
 *   PA4=CS0(soft)  -> MT6825 #0
 *   PB12=CS1(soft) -> MT6825 #1
 *
 *   USART2 PA3=RX  460800 <- VB22A #0 TX
 *   USART2 PA2=TX  460800 -> VB22A #0 RX
 *   USART3 PB11=RX  460800 <- VB22A #1 TX
 *   USART3 PB10=TX  460800 -> VB22A #1 RX
 *
 *   USART1 PA9=TX  115200 -> Jetson
 *
 *   TIM2  CH1=PA0  CH2=PA1 -> Motor #0 PWM
 *   TIM3  CH1=PB4  CH2=PB5 -> Motor #1 PWM
 *
 *   PC13   LED debug
 *
 * Packet -> Jetson (14 bytes):
 *   [0]      0xAA  header
 *   [1]      id   lidar index (0 or 1)
 *   [2][3]   dist_mm   uint16 LE   (0xFFFF=invalid)
 *   [4][5]   angle_deg10  int16 LE
 *   [6..9]   timestamp_ms uint32 LE
 *   [10][11] position_lo  uint32 lo (debug encoder raw)
 *   [12]     checksum XOR [1..11]
 *   [13]     0x55  footer
 */
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <math.h>

#define MT6825_COUNT     2
#define MT6825_CS_PORT0  GPIOA
#define MT6825_CS_PIN0   GPIO_PIN_4
#define MT6825_CS_PORT1  GPIOB
#define MT6825_CS_PIN1   GPIO_PIN_12
#define MT6825_RES       262144UL   /* 2^18 */

#define VB22A_COUNT      2
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_MM     20000
#define VB22A_MIN_MM     10

#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     14

#define MOTOR_COUNT   2
#define PWM_MAX       900
#define PWM_MIN       150
#define KP_MOTOR      0.005f   /* pwm = error_raw * KP */
#define DEADBAND_RAW  500      /* ~0.7 deg deadband */

#define SCAN_RAW_HALF  65536L   /* 90 degrees in raw units */

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;  /* -> Jetson 115200 */
extern UART_HandleTypeDef huart2;  /* <-> VB22A #0 460800 */
extern UART_HandleTypeDef huart3;  /* <-> VB22A #1 460800 */
extern TIM_HandleTypeDef  htim2;   /* Motor #0 PWM */
extern TIM_HandleTypeDef  htim3;   /* Motor #1 PWM */

void lidar_encoder2_init(void);
void lidar_encoder2_tick(void);
