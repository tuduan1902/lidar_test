/**
 * front_lidar_stm32.h
 * ============================================================
 * CUM 5 LiDAR VB22A DAU XE MAY - STM32F407VET6
 *
 * THAY DOI SO VOI PHIEN BAN TRUOC:
 *   - HAL_Delay(5) trong while(1) -> TIM6 ngat 5ms
 *     CPU khong bi block, while(1) de trong hoan toan
 *   - HAL_UART_Transmit blocking -> HAL_UART_Transmit_DMA
 *     CPU khong doi khi gui packet len Jetson
 *   - RX van dung IT (4 bytes/packet, can parse header byte-by-byte)
 *   - Them co tx_busy tranh chong cheo DMA TX
 *
 * CUBEMX SETUP (bo sung so voi truoc):
 *   1. Timers -> TIM6:
 *      - Activated: checked
 *      - Prescaler: 83  (84MHz / (83+1) = 1MHz)
 *      - Counter Period: 4999  (1MHz / (4999+1) = 200Hz... sai)
 *        Thuc ra: 1MHz / 5000 = 200Hz moi tick = 5ms. DUNG.
 *      - auto-reload preload: Enable
 *      - NVIC: TIM6 global interrupt -> Enable
 *
 *   2. USART6 (Uplink Jetson):
 *      - Mode: Asynchronous
 *      - Baud: 460800, 8N1
 *      - DMA Settings -> Add -> USART6_TX
 *        Direction: Memory to Peripheral
 *        Mode: Normal (khong phai Circular)
 *        Data Width: Byte/Byte
 *      - NVIC: USART6 global interrupt -> Enable
 *        (de HAL_UART_TxCpltCallback chay duoc)
 *
 *   3. USART1/2/3, UART4/5 (LiDAR sensors):
 *      - Mode: Asynchronous, Baud: 460800, 8N1
 *      - NVIC: global interrupt -> Enable (cho IT RX)
 *      - KHONG can DMA cho RX (4 bytes/packet)
 *
 * THEM VAO main.c:
 *   Includes  : #include "front_lidar_stm32.h"
 *   USER CODE 2: FrontLidar_Init();
 *   while(1)  : de trong (khong can gi ca)
 *   USER CODE 4:
 *     void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *         FrontLidar_UART_RxCallback(huart);
 *     }
 *     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *         FrontLidar_TIM_Callback(htim);
 *     }
 *
 * PROTOCOL VB22A (4 bytes):
 *   [0]=0x5C  [1]=dist_L  [2]=dist_H  [3]=~(dist_L+dist_H)
 *   dist don vi MM, baudrate 460800
 *   Lenh start: 5A 0A 02 02 00 F1
 *
 * PACKET UPLINK (12 bytes, header 0xCC):
 *   [0]=0xCC [1]=id(0-4) [2,3]=dist_mm [4,5]=angle_10
 *   [6,7]=ox_mm [8,9]=oy_mm [10]=XOR[1..9] [11]=0x55
 * ============================================================
 */
#ifndef FRONT_LIDAR_STM32_H
#define FRONT_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PROTOCOL VB22A
 * ============================================================ */
#define VB22A_PKT_LEN       4
#define VB22A_HEADER        0x5C
#define VB22A_CMD_LEN       6
#define VB22A_OUT_OF_RANGE  20000
#define VB22A_DIST_MIN_MM   50
#define VB22A_DIST_MAX_MM   19999

static const uint8_t VB22A_CMD_START[VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x02,0x00,0xF1};
static const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x00,0x00,0xF3};

/* ============================================================
 * PACKET UPLINK
 * ============================================================ */
#define FRONT_PKT_LEN       12
#define FRONT_PKT_HDR       0xCC
#define FRONT_PKT_FTR       0x55
#define FRONT_N_LIDAR       5
#define FRONT_ID_TILT       4

/* ============================================================
 * CAU HINH VI TRI SENSOR (sua theo thuc te)
 * ============================================================ */
typedef struct {
    float angle_deg;
    float ox_m;
    float oy_m;
} FrontLidarMount;

static const FrontLidarMount FRONT_LIDAR_MOUNTS[FRONT_N_LIDAR] = {
    /* id=0 F0 ngoai trai  thang truoc */ { 90.0f, -0.30f, +0.90f },
    /* id=1 F1 trong trai  thang truoc */ { 90.0f, -0.15f, +0.60f },
    /* id=2 F2 trong phai  thang truoc */ { 90.0f, +0.15f, +0.60f },
    /* id=3 F3 ngoai phai  thang truoc */ { 90.0f, +0.30f, +0.90f },
    /* id=4 F4 nghieng quet o ga       */ { 90.0f,  0.00f, +0.80f },
};

/* ============================================================
 * UART MAPPING
 * ============================================================ */
extern UART_HandleTypeDef huart1; /* F0 */
extern UART_HandleTypeDef huart2; /* F1 */
extern UART_HandleTypeDef huart3; /* F2 */
extern UART_HandleTypeDef huart4; /* F3 */
extern UART_HandleTypeDef huart5; /* F4 nghieng */
extern UART_HandleTypeDef huart6; /* Uplink Jetson - DMA TX */
extern TIM_HandleTypeDef  htim6;  /* Timer 5ms trigger Process */

#define FRONT_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4, &huart5 }

/* ============================================================
 * TRANG THAI SENSOR
 * ============================================================ */
typedef struct {
    uint8_t  pkt[VB22A_PKT_LEN];
    uint8_t  pkt_idx;
    uint16_t dist_mm;
    bool     valid;
    uint32_t ts_ms;
} FrontLidarState;

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/* Goi 1 lan trong USER CODE BEGIN 2 cua main.c */
void FrontLidar_Init(void);

/* Goi trong HAL_UART_RxCpltCallback cua main.c */
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart);

/* Goi trong HAL_TIM_PeriodElapsedCallback cua main.c
 * TIM6 ngat moi 5ms, ham nay quet 5 sensor va gui DMA */
void FrontLidar_TIM_Callback(TIM_HandleTypeDef* htim);

/* Goi trong HAL_UART_TxCpltCallback khi USART6 DMA TX hoan thanh
 * if (huart->Instance == USART6) { FrontLidar_TxDone(); } */
void FrontLidar_TxDone(void);

#endif /* FRONT_LIDAR_STM32_H */