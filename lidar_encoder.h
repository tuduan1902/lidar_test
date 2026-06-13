/**
 * lidar_fixed.h
 * STM32 firmware cho LiDAR VB22A 1 tia cố định
 * Giao tiếp: VB22A (USART2 460800) -> Packet 10 bytes -> Jetson (USART1 115200)
 */
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ---- VB22A ---- */
#define VB22A_HEADER     0x5C
#define VB22A_FRAME_LEN  4
#define VB22A_MAX_MM     20000
#define VB22A_MIN_MM     10

/* ---- Packet -> Jetson (10 bytes) ---- */
/*
 * [0]     0xAA (Header)
 * [1][2]  dist_mm (uint16_t Little Endian)
 * [3..6]  timestamp_ms (uint32_t Little Endian)
 * [7]     0x00 (Padding byte)
 * [8]     Checksum (XOR từ byte 1 đến byte 7)
 * [9]     0x55 (Footer)
 */
#define PKT_HEADER  0xAA
#define PKT_FOOTER  0x55
#define PKT_LEN     10

void lidar_fixed_init(void);
void lidar_fixed_tick(void);