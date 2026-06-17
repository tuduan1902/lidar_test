/**
 * front_lidar_stm32.h
 * ============================================================
 * CUM 5 LiDAR VB22A DAU XE MAY - STM32F407VET6
 *
 * DEBUG LED: PA6
 *   - Nhay nhanh (100ms): TIM6 dang chay, he thong hoat dong
 *   - Nhay cham (500ms): TIM6 chay nhung KHONG co data tu sensor
 *   - Sang lien tuc: DMA TX dang busy (s_tx_busy stuck)
 *   - Tat: chua vao Init hoac bi loi UART ErrorCallback
 *
 * BUG DA SUA:
 *   1. Bo "break" trong TIM6 callback: gio gui tat ca sensor co
 *      data trong 1 ISR, khong chi 1 sensor.
 *   2. DMA busy: dung queue 5 slot thay vi bo qua packet.
 *      Neu DMA ban, packet duoc giu lai va gui o ISR ke tiep.
 *   3. Lenh Start Ranging: gui blocking 1 lan trong Init, OK.
 *
 * CUBEMX (giong truoc, khong thay doi):
 *   USART1-5 @ 460800, NVIC enable (sensor RX IT)
 *   USART6   @ 460800, DMA2 Stream6 TX, NVIC enable (uplink)
 *   TIM6: Prescaler=83, Period=4999 -> 5ms, NVIC enable
 *   PA6: GPIO_Output (LED debug)
 *
 * THEM VAO main.c:
 *   #include "front_lidar_stm32.h"
 *   USER CODE 2: FrontLidar_Init();
 *   USER CODE 4:
 *     void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
 *         FrontLidar_UART_RxCallback(h);
 *     }
 *     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h) {
 *         FrontLidar_TIM_Callback(h);
 *     }
 *     void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) {
 *         if (h->Instance == USART6) FrontLidar_TxDone();
 *     }
 *     void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
 *         FrontLidar_UART_ErrorCallback(h);
 *     }
 * ============================================================
 */
#ifndef FRONT_LIDAR_STM32_H
#define FRONT_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * DEBUG LED (PA6 - co san tren board)
 * Comment dong #define nay de tat debug LED khi deploy
 * ============================================================ */
#define FRONT_DEBUG_LED_ENABLE
#define FRONT_DEBUG_LED_PORT    GPIOA
#define FRONT_DEBUG_LED_PIN     GPIO_PIN_6

/* ============================================================
 * PROTOCOL VB22A (4 bytes, datasheet V3.0)
 * ============================================================ */
#define VB22A_PKT_LEN       4
#define VB22A_HEADER        0x5C
#define VB22A_CMD_LEN       6
#define VB22A_OUT_OF_RANGE  20000
#define VB22A_DIST_MIN_MM   50
#define VB22A_DIST_MAX_MM   19999

/* Khai bao extern - dinh nghia trong front_lidar_stm32.c */
extern const uint8_t VB22A_CMD_START[VB22A_CMD_LEN];
extern const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN];

/* ============================================================
 * PACKET UPLINK (12 bytes, STM32 -> Jetson)
 * [0]=0xCC [1]=id [2,3]=dist_mm [4,5]=angle_10
 * [6,7]=ox_mm [8,9]=oy_mm [10]=XOR[1..9] [11]=0x55
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

/* Dinh nghia extern - thuc su khai bao trong front_lidar_stm32.c */
extern const FrontLidarMount FRONT_LIDAR_MOUNTS[FRONT_N_LIDAR];

/* ============================================================
 * UART MAPPING
 * ============================================================ */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef  htim6;

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
 * BIEN DEBUG (doc tu Watch window Keil)
 * ============================================================ */
extern volatile uint32_t g_dbg_rx_bytes;      /* tong byte nhan tu tat ca sensor */
extern volatile uint32_t g_dbg_pkt_ok;        /* tong packet VB22A checksum dung */
extern volatile uint32_t g_dbg_pkt_bad_chk;   /* packet sai checksum */
extern volatile uint32_t g_dbg_tx_sent;       /* tong packet da gui qua DMA */
extern volatile uint32_t g_dbg_tx_busy_skip;  /* so lan DMA ban, phai hoan gui */
extern volatile uint32_t g_dbg_tim6_tick;     /* so lan TIM6 ngat */
extern volatile uint8_t  g_dbg_sensor_data[FRONT_N_LIDAR]; /* 1=sensor co data */
extern volatile uint32_t g_dbg_uart_err;      /* so lan UART ErrorCallback */

/* ============================================================
 * PUBLIC API
 * ============================================================ */
void FrontLidar_Init(void);
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart);
void FrontLidar_TIM_Callback(TIM_HandleTypeDef* htim);
void FrontLidar_TxDone(void);
void FrontLidar_UART_ErrorCallback(UART_HandleTypeDef* huart);

#endif /* FRONT_LIDAR_STM32_H */