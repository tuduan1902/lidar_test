/**
 * dual_lidar_encoder.h
 * STM32F103C8T6 -- 2x VB22A + 2x MT6825 + 2x motor -> Jetson
 *
 * Mo rong tu lidar_with_encoder.h (1 lidar) len 2 lidar
 * Giu nguyen logic hoan toan, chi nhan doi sang array [2]
 *
 * ============================================================
 * PIN MAP
 * ============================================================
 *
 *  USART1  PA9(TX)  PA10(RX)  115200   -> Jetson (khong doi)
 *  USART2  PA2(TX)  PA3(RX)   460800   <- VB22A LEFT  (id=0)
 *  USART3  PB10(TX) PB11(RX)  460800   <- VB22A RIGHT (id=1)
 *
 *  SPI1    PA5=SCK  PA6=MISO  PA7=MOSI  (chung 2 encoder)
 *          PA4 = CS encoder LEFT   (id=0) -- khong doi
 *          PB0 = CS encoder RIGHT  (id=1) -- them moi
 *
 *  TIM2    PA0=CH1(FWD)  PA1=CH2(REV)  -> Motor LEFT  (id=0) -- khong doi
 *  TIM3    PA6=CH1(FWD)  PA7=CH2(REV)  -> Motor RIGHT (id=1) -- them moi
 *          NOTE: PA6/PA7 trung SPI1_MISO/MOSI!
 *          -> Dung pin remap TIM3:
 *             TIM3 CH1 remap = PB4, CH2 remap = PB5
 *             Trong CubeMX: enable TIM3 partial remap
 *             PB4=TIM3_CH1(FWD_RIGHT)  PB5=TIM3_CH2(REV_RIGHT)
 *
 * ============================================================
 * PACKET -> Jetson (14 bytes, giong cu):
 *   [0]      0xAA  header
 *   [1]      id    0=LEFT 1=RIGHT  <-- thay doi: truoc la luon 0x00
 *   [2][3]   dist_mm   uint16 LE
 *   [4][5]   angle_deg10  int16 LE
 *   [6..9]   timestamp_ms uint32 LE
 *   [10][11] position_lo  uint32 LE
 *   [12]     checksum XOR [1..11]
 *   [13]     0x55  footer
 *
 * ============================================================
 * CUBEMX CONFIG (them so voi file cu):
 * ============================================================
 *   USART3: 460800 8N1  PB10=TX PB11=RX  (them moi)
 *   GPIO:   PB0=Output  (CS encoder RIGHT, them moi)
 *   TIM3:   PWM Partial Remap
 *           CH1=PB4(FWD_RIGHT)  CH2=PB5(REV_RIGHT)
 *           PSC=71 ARR=999 -> 1kHz  (giong TIM2)
 *   Giu nguyen: SPI1, USART1, USART2, TIM2, PA4(CS_LEFT), PC13(LED)
 */

#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <math.h>

/* ---- So luong LiDAR ---- */
#define NUM_LIDAR    2
#define LIDAR_LEFT   0
#define LIDAR_RIGHT  1

/* ---- MT6825 ---- */
#define MT6825_RES   262144UL   /* 2^18 */

/* CS pins */
#define CS_LEFT_PORT   GPIOA
#define CS_LEFT_PIN    GPIO_PIN_4   /* PA4 -- khong doi */
#define CS_RIGHT_PORT  GPIOB
#define CS_RIGHT_PIN   GPIO_PIN_0   /* PB0 -- them moi  */

/* ---- VB22A ---- */
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_MM     20000
#define VB22A_MIN_MM     10

/* ---- Packet ---- */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     14

/* ---- Motor ---- */
#define PWM_MAX      900
#define PWM_MIN      150
#define KP_MOTOR     0.005f
#define DEADBAND_RAW 500    /* ~0.7 deg */

/* ---- Goc quet: +-90 deg ---- */
/* 90 deg = 90/360 * 262144 = 65536 raw */
#define SCAN_RAW_HALF  65536L

/* ---- API ---- */
void dual_lidar_init(void);
void dual_lidar_tick(void);