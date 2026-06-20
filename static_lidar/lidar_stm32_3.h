/**
 * rear_lidar_stm32.h  — Cau hinh 3 sensor sau xe
 * TIM7 ngat 5ms
 * USART6 DMA TX uplink Jetson @ 460800
 */
#ifndef REAR_LIDAR_STM32_H
#define REAR_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PROTOCOL VB22A (dung chung voi front)
 * ============================================================ */
#define VB22A_PKT_LEN        4
#define VB22A_HEADER         0x5C
#define VB22A_CMD_LEN        6
#define VB22A_OUT_OF_RANGE   20000
#define VB22A_DIST_MIN_MM    50
#define VB22A_DIST_MAX_MM    19999

static const uint8_t VB22A_CMD_START[VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x02,0x00,0xF1};
static const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x00,0x00,0xF3};

/* ============================================================
 * PACKET UPLINK (12 bytes, header 0xBB)
 * ============================================================ */
#define REAR_PKT_LEN         12
#define REAR_PKT_HDR         0xBB
#define REAR_PKT_FTR         0x55

// ĐÃ SỬA: Giảm từ 5 xuống 3 LiDAR
#define REAR_N_LIDAR         3

/* ============================================================
 * CAU HINH VI TRI (3 LiDAR)
 * angle=180 do = thang ra sau (he toa do xe: +Y truoc, +X phai)
 * ============================================================ */
typedef struct {
    float angle_deg;
    float ox_m;
    float oy_m;
} RearLidarMount;

static const RearLidarMount REAR_LIDAR_MOUNTS[REAR_N_LIDAR] = {
    /* id=0 L0 thang sau    */ { 180.0f,  0.00f, -0.85f },
    /* id=1 L1 -45 do Trai  */ { 135.0f, -0.30f, -0.85f },
    /* id=2 L2 +45 do Phai  */ { 225.0f, +0.30f, -0.85f },
};

/* ============================================================
 * UART MAPPING (Bỏ UART4 và UART5)
 * ============================================================ */
extern UART_HandleTypeDef huart1; /* L0 */
extern UART_HandleTypeDef huart2; /* L1 */
extern UART_HandleTypeDef huart3; /* L2 */
extern UART_HandleTypeDef huart6; /* Uplink Jetson DMA TX */
extern TIM_HandleTypeDef  htim7;  /* Timer 5ms */

// ĐÃ SỬA: Chỉ map 3 cổng UART
#define REAR_LIDAR_UART_LIST { &huart1, &huart2, &huart3 }

/* ============================================================
 * TRANG THAI
 * ============================================================ */
typedef struct {
    uint8_t  pkt[VB22A_PKT_LEN];
    uint8_t  pkt_idx;
    uint16_t dist_mm;
    bool     valid;
    uint32_t ts_ms;
} RearLidarState;

/* ============================================================
 * PUBLIC API
 * ============================================================ */
void RearLidar_Init(void);
void RearLidar_UART_RxCallback(UART_HandleTypeDef* huart);
void RearLidar_TIM_Callback(TIM_HandleTypeDef* htim);
void RearLidar_TxDone(void);

#endif /* REAR_LIDAR_STM32_H */