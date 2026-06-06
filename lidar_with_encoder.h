/**
 * lidar_with_encoder.h
 * STM32F103C8T6
 * MT6825 (SPI1) + VB22A (USART2 460800) -> Packet -> Jetson (USART1 115200)
 *
 * PIN MAP:
 *   SPI1  PA5=SCK  PA6=MISO  PA7=MOSI  PA4=CS(soft)  -> MT6825
 *   USART2 PA3=RX  460800 baud   <- VB22A TX
 *   USART2 PA2=TX  460800 baud   -> VB22A RX (gui lenh start)
 *   USART1 PA9=TX  115200 baud   -> Jetson
 *   TIM2   CH1=PA0  CH2=PA1      -> Motor PWM (Forward/Reverse)
 *   PC13   LED debug
 *
 * PACKET -> Jetson (14 bytes):
 *   [0]      0xAA  header
 *   [1]      0x00  id
 *   [2][3]   dist_mm   uint16 LE   (0xFFFF=invalid)
 *   [4][5]   angle_deg10  int16 LE (goc*10, -900=-90.0 deg)
 *   [6..9]   timestamp_ms uint32 LE
 *   [10][11] position_lo  uint32 lo (debug encoder raw)
 *   [12]     checksum XOR [1..11]
 *   [13]     0x55  footer
 *
 * MOTOR QUET:
 *   Dung position tuyet doi (int64) de dieu khien
 *   Moi vong = 262144 raw units
 *   Quet tu -90 den +90 do, doi chieu lien tuc
 *   -90 deg = -65536 raw units (tu vi tri 0)
 *   +90 deg = +65536 raw units
 */
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <math.h>

/* ---- MT6825 ---- */
#define MT6825_CS_PORT   GPIOA
#define MT6825_CS_PIN    GPIO_PIN_4
#define MT6825_RES       262144UL   /* 2^18 */

/* ---- VB22A ---- */
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_CM     20000
#define VB22A_MIN_CM     10

/* ---- Packet ---- */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     14

/* ---- Motor ---- */
#define PWM_MAX      900
#define PWM_MIN      150
#define KP_MOTOR     0.005f   /* pwm = error_raw * KP */
#define DEADBAND_RAW 500      /* ~0.7 deg deadband */

/* ---- Quet goc ---- */
/* 90 deg = 90/360 * 262144 = 65536 raw */
#define SCAN_RAW_HALF  65536L   /* tuong duong 90 deg */

void lidar_encoder_init(void);
void lidar_encoder_tick(void);