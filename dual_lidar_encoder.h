/**
 * dual_lidar_encoder.h — DMA version
 * STM32F103C8T6: 2x VB22A + 2x MT6825 + 2x motor
 *
 * THAY DOI so voi blocking version:
 *   USART2 + USART3 dung DMA RX circular buffer
 *   -> Khong block tick(), 2 lidar doc song song
 *
 * PIN MAP (khong doi):
 *   USART1  PA9/PA10   115200  -> Jetson
 *   USART2  PA2/PA3    460800  <- VB22A LEFT  (id=0)  + DMA1 CH6
 *   USART3  PB10/PB11  460800  <- VB22A RIGHT (id=1)  + DMA1 CH3
 *   SPI1    PA5/PA6/PA7        -> MT6825 x2
 *           PA4=CS_LEFT  PB0=CS_RIGHT
 *   TIM2    PA0/PA1    motor LEFT  CH1/CH2
 *   TIM3    PB4/PB5    motor RIGHT CH1/CH2 (partial remap)
 *   PC13    LED
 *
 * DMA config trong CubeMX:
 *   USART2 RX: DMA1 Channel6, Circular, Byte
 *   USART3 RX: DMA1 Channel3, Circular, Byte
 *
 * PACKET -> Jetson (14 bytes):
 *   [0]     0xAA
 *   [1]     id (0=LEFT 1=RIGHT)
 *   [2][3]  dist_mm uint16 LE
 *   [4][5]  angle_deg*10 int16 LE
 *   [6..9]  timestamp uint32 LE
 *   [10][11] pos_lo uint16 LE
 *   [12]    checksum XOR[1..11]
 *   [13]    0x55
 */
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <math.h>

#define NUM_LIDAR     2
#define LIDAR_LEFT    0
#define LIDAR_RIGHT   1

/* MT6825 */
#define MT6825_RES       262144UL
#define CS_LEFT_PORT     GPIOA
#define CS_LEFT_PIN      GPIO_PIN_4
#define CS_RIGHT_PORT    GPIOB
#define CS_RIGHT_PIN     GPIO_PIN_0

/* VB22A frame */
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_MM     20000
#define VB22A_MIN_MM     10

/* DMA circular buffer: >=4 frame = 16 bytes */
#define DMA_BUF_LEN      32

/* Packet */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     14

/* Motor */
#define PWM_MAX       900
#define PWM_MIN       150
#define KP_MOTOR      0.005f
#define DEADBAND_RAW  500
#define SCAN_RAW_HALF 65536L   /* 90 deg */

void dual_lidar_init(void);
void dual_lidar_tick(void);

/* Goi trong stm32f1xx_it.c hoac main:
 * extern DMA_HandleTypeDef hdma_usart2_rx;
 * extern DMA_HandleTypeDef hdma_usart3_rx;
 */