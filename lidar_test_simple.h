/**
 * lidar_test_simple.h
 * STM32F103C8T6 -- VB22A, dung protocol dung tu datasheet V3.0
 *
 * DAU DAY:
 *   VB22A VCC  -> Blue Pill 3.3V
 *   VB22A GND  -> GND
 *   VB22A TX   -> PA3 (USART2_RX)  460800 baud
 *   VB22A RX   <- PA2 (USART2_TX)  460800 baud  <- PHAI NOI de gui lenh
 *
 *   Blue Pill PA9 (USART1_TX) -> Jetson pin 10 (THS1_RX)  115200 baud
 *   Blue Pill GND             -> Jetson GND
 *
 * KEIL USART CONFIG:
 *   USART1: 115200 baud, 8N1  (-> Jetson)
 *   USART2: 460800 baud, 8N1  (<-> VB22A)
 *
 * FRAME VB22A OUTPUT (4 bytes):
 *   [0]    0x5C  header
 *   [1][2] dist (2 byte, little-endian, don vi mm)
 *          Range: 0~20000mm, out-of-range = 20000
 *   [3]    checksum = ~(sum(bytes[1..2])) & 0xFF
 *
 * LENH GUI VB22A:
 *   Start Ranging: 5A 0A 02 02 F1
 *   Stop  Ranging: 5A 0A 02 00 F3
 */

#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* VB22A frame */
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_MM     20000
#define VB22A_MIN_MM     10

/* Packet STM32 -> Jetson (10 bytes):
 * [0]      0xAA  header
 * [1]      0x00  lidar id
 * [2][3]   dist_mm uint16 LE  (0xFFFF = invalid)
 * [4..7]   timestamp_ms uint32 LE
 * [8]      checksum XOR bytes[1..7]
 * [9]      0x55  footer
 */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     10

void lidar_test_init(void);
void lidar_test_tick(void);