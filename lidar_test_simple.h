/**
 * lidar_test_simple.h
 * STM32F103C8T6 -- Test VB22A cam tay, KHONG encoder, KHONG motor
 *
 * ============================================================
 * SPEC VB22A (Vermilion Bird Series VB22A, datasheet V3.0)
 * ============================================================
 *   Loai        : Single-point LiDAR (1 diem do moi frame)
 *   Tan so do   : 200 Hz
 *   Tam do      : 0.1m ~ 20m
 *   Do chinh xac: +/- 2cm (<10m),  +/- 1% (10~20m)
 *   Giao tiep   : UART TTL 3.3V
 *   Baud rate   : mac dinh 115200, TOI DA 460800
 *   Frame output: 9 bytes continuous (khong can trigger)
 *   Nguon       : 5V, ~120mA
 *
 * FRAME VB22A (9 bytes):
 *   [0]  0x59  header1
 *   [1]  0x59  header2
 *   [2]  dist_L     khoang cach (cm), little-endian
 *   [3]  dist_H
 *   [4]  strength_L signal strength (bo qua)
 *   [5]  strength_H
 *   [6]  temp_L     nhiet do chip (bo qua)
 *   [7]  temp_H
 *   [8]  checksum = (sum bytes[0..7]) & 0xFF  <-- CONG, khong XOR
 *
 * ============================================================
 * DAU DAY (5 day):
 * ============================================================
 *   VB22A VCC ->  Blue Pill 5V
 *   VB22A GND ->  GND
 *   VB22A TX  ->  PA3 (USART2_RX)  115200 baud
 *   VB22A RX  <-  PA2 (USART2_TX)  co the bo qua
 *
 *   Blue Pill PA9 (USART1_TX) -> Pi GPIO15/RXD (pin 10)  460800 baud
 *   Blue Pill GND             -> Pi GND (pin 6)
 *   HOAC: cam USB Micro Blue Pill -> Pi, dung /dev/ttyACM0
 *
 * ============================================================
 * KEIL MDK -- USART CONFIG:
 * ============================================================
 *   USART1: Baud=460800  8N1  PA9(TX) PA10(RX)
 *   USART2: Baud=115200  8N1  PA2(TX) PA3(RX)
 */

#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ---- VB22A ---- */
#define VB22A_HEADER     0x59
#define VB22A_FRAME_LEN  9
#define VB22A_MAX_CM     2000    /* 20m */
#define VB22A_MIN_CM     10     /* 0.1m, duoi nay la nhieu */

/* ---- Packet STM32 -> Pi (10 bytes) ----
 * [0]    0xAA  header
 * [1]    0x00  lidar id
 * [2][3] dist_mm uint16 LE  (0xFFFF = invalid)
 * [4..7] timestamp_ms uint32 LE
 * [8]    checksum = XOR bytes[1..7]
 * [9]    0x55  footer
 */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     10

void lidar_test_init(void);
void lidar_test_tick(void);