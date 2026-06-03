/**
 * lidar_test_simple.h
 * STM32F103C8T6 — Test VB22A cam tay, KHONG encoder, KHONG motor
 *
 * DAU DAY (chi 5 day):
 *   VB22A VCC  ->  Blue Pill 5V
 *   VB22A GND  ->  GND
 *   VB22A TX   ->  PA3 (USART2_RX)
 *   VB22A RX   <-  PA2 (USART2_TX)   co the bo qua
 *
 *   Blue Pill PA9 (USART1_TX)  ->  Pi pin 10 (GPIO15/RXD)
 *   Blue Pill GND              ->  Pi pin 6  (GND)
 *   HOAC cam USB Micro -> Pi, dung /dev/ttyACM0
 */

#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define VB22A_HEADER    0x59
#define VB22A_FRAME_LEN 9
#define VB22A_MAX_CM    2000

/* Packet 10 bytes:
 * [0]      0xAA  header
 * [1]      0x00  id
 * [2][3]   dist_mm uint16 LE  (0xFFFF = invalid)
 * [4..7]   timestamp_ms uint32 LE
 * [8]      XOR bytes[1..7]
 * [9]      0x55  footer
 */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     10