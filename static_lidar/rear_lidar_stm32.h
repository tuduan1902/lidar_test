/**
 * rear_lidar_stm32.h
 * ============================================================
 * CAU HINH 5 LiDAR VB22A gan sau duoi xe may, bo tri vong cung 90°
 *
 * PROTOCOL VB22A DUNG (4 bytes, tu datasheet V3.0):
 *   Baudrate: 460800
 *   [0] 0x5C  header
 *   [1] dist_L, [2] dist_H  -> dist_mm = dist_H<<8|dist_L (don vi MM)
 *   [3] checksum = ~(dist_L + dist_H) & 0xFF
 *   Out-of-range: sensor tra ve 20000
 *   Lenh khoi dong: 5A 0A 02 02 00 F1 (Start Ranging, 6 bytes)
 *
 * PACKET UPLINK (12 bytes, STM32 -> Jetson):
 *   [0]    = 0xBB  header
 *   [1]    = id    (0-4)
 *   [2,3]  = dist_mm  uint16 LE (MM)
 *   [4,5]  = angle_10 int16  LE (goc x10 do)
 *   [6,7]  = ox_mm    int16  LE
 *   [8,9]  = oy_mm    int16  LE
 *   [10]   = checksum XOR byte[1..9]
 *   [11]   = 0x55  footer
 *
 * SO DO BO TRI (nhin tu tren xuong, mui xe huong len tren):
 *
 *         [mui xe]
 *   ---------+--------- tam xe
 *            |
 *   [L2][L1][L0][L3][L4]   <- duoi xe
 *    \   \   |   /   /
 *   135 157.5 180 202.5 225 (do so voi truc +X)
 *
 *   L0: thang ra sau    (angle=180, ox= 0.00, oy=-0.85)
 *   L1: -22.5 do trai   (angle=157.5, ox=-0.15, oy=-0.85)
 *   L2: -45  do trai    (angle=135,   ox=-0.30, oy=-0.85)
 *   L3: +22.5 do phai   (angle=202.5, ox=+0.15, oy=-0.85)
 *   L4: +45  do phai    (angle=225,   ox=+0.30, oy=-0.85)
 *
 * HARDWARE:
 *   L0->USART1, L1->USART2, L2->USART3, L3->UART4, L4->UART5
 *   Uplink->USART6 @ 460800 -> USB-UART -> /dev/ttyUSB2
 * ============================================================
 */
#ifndef REAR_LIDAR_STM32_H
#define REAR_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define VB22A_PKT_LEN        4
#define VB22A_HEADER         0x5C
#define VB22A_CMD_LEN        6
#define VB22A_OUT_OF_RANGE   20000
#define VB22A_DIST_MIN_MM    50
#define VB22A_DIST_MAX_MM    19999

static const uint8_t VB22A_CMD_START[VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x02,0x00,0xF1};
static const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x00,0x00,0xF3};

#define REAR_PKT_LEN         12
#define REAR_PKT_HDR         0xBB
#define REAR_PKT_FTR         0x55
#define REAR_N_LIDAR         5

typedef struct {
    float angle_deg;
    float ox_m;
    float oy_m;
} RearLidarMount;

static const RearLidarMount REAR_LIDAR_MOUNTS[REAR_N_LIDAR] = {
    /* id=0 L0 thang sau    */ { 180.0f,  0.00f, -0.85f },
    /* id=1 L1 -22.5 do T   */ { 157.5f, -0.15f, -0.85f },
    /* id=2 L2 -45  do T    */ { 135.0f, -0.30f, -0.85f },
    /* id=3 L3 +22.5 do P   */ { 202.5f, +0.15f, -0.85f },
    /* id=4 L4 +45  do P    */ { 225.0f, +0.30f, -0.85f },
};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

#define REAR_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4, &huart5 }

typedef struct {
    uint8_t  pkt[VB22A_PKT_LEN];
    uint8_t  pkt_idx;
    uint16_t dist_mm;
    bool     valid;
    uint32_t ts_ms;
} RearLidarState;

void RearLidar_Init(void);
void RearLidar_Process(void);
void RearLidar_UART_RxCallback(UART_HandleTypeDef* huart);

#endif /* REAR_LIDAR_STM32_H */