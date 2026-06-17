/**
 * rear_lidar_stm32.h  — giong front nhung 5 sensor sau xe
 * TIM7 ngat 5ms (de khong xung dot voi TIM6 cua front neu dung chung board)
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
#define REAR_N_LIDAR         5

/* ============================================================
 * CUBEMX SETUP (rear board):
 *   TIM7: Prescaler=83, Period=4999 -> ngat 5ms, enable NVIC
 *   USART1-5 @ 460800, enable NVIC IT
 *   USART6 @ 460800, DMA TX (Memory->Peripheral, Normal), enable NVIC
 *
 * THEM VAO main.c:
 *   #include "rear_lidar_stm32.h"
 *   USER CODE 2: RearLidar_Init();
 *   USER CODE 4:
 *     void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *         RearLidar_UART_RxCallback(huart);
 *     }
 *     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *         RearLidar_TIM_Callback(htim);
 *     }
 *     void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *         if (huart->Instance == USART6) { RearLidar_TxDone(); }
 *     }
 * ============================================================ */

/* ============================================================
 * CAU HINH VI TRI (sua theo thuc te)
 * angle=180 do = thang ra sau (he toa do xe: +Y truoc, +X phai)
 * ============================================================ */
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

/* ============================================================
 * UART MAPPING
 * ============================================================ */
extern UART_HandleTypeDef huart1; /* L0 */
extern UART_HandleTypeDef huart2; /* L1 */
extern UART_HandleTypeDef huart3; /* L2 */
extern UART_HandleTypeDef huart4; /* L3 */
extern UART_HandleTypeDef huart5; /* L4 */
extern UART_HandleTypeDef huart6; /* Uplink Jetson DMA TX */
extern TIM_HandleTypeDef  htim7;  /* Timer 5ms */

#define REAR_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4, &huart5 }

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