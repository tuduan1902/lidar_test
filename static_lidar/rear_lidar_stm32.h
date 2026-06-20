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

#define REAR_DEBUG_LED_ENABLE
#define REAR_DEBUG_LED_PORT    GPIOA
#define REAR_DEBUG_LED_PIN     GPIO_PIN_6

/* ============================================================
 * PROTOCOL VB22A (dung chung voi front)
 * ============================================================ */
#define VB22A_PKT_LEN        4
#define VB22A_HEADER         0x5C
#define VB22A_CMD_LEN        6
#define VB22A_OUT_OF_RANGE   20000
#define VB22A_DIST_MIN_MM    50
#define VB22A_DIST_MAX_MM    19999

extern const uint8_t VB22A_CMD_START[VB22A_CMD_LEN];
extern const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN];

/* ============================================================
 * PACKET UPLINK (12 bytes, header 0xBB)
 * ============================================================ */
#define REAR_PKT_LEN         12
#define REAR_PKT_HDR         0xBB
#define REAR_PKT_FTR         0x55
#define REAR_N_LIDAR         3

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

extern const RearLidarMount REAR_LIDAR_MOUNTS[REAR_N_LIDAR];


/* ============================================================
 * UART MAPPING
 * ============================================================ */
extern UART_HandleTypeDef huart1; /* L0 */
extern UART_HandleTypeDef huart2; /* L1 */
extern UART_HandleTypeDef huart3; /* L2 */
extern UART_HandleTypeDef huart6; /* Uplink Jetson DMA TX */
extern TIM_HandleTypeDef  htim6;  /* Timer 5ms */

#define REAR_LIDAR_UART_LIST { &huart1, &huart2, &huart3}

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
 * BIEN DEBUG (doc tu Watch window Keil)
 * ============================================================ */
extern volatile uint32_t g_dbg_rx_bytes;      /* tong byte nhan tu tat ca sensor */
extern volatile uint32_t g_dbg_pkt_ok;        /* tong packet VB22A checksum dung */
extern volatile uint32_t g_dbg_pkt_bad_chk;   /* packet sai checksum */
extern volatile uint32_t g_dbg_tx_sent;       /* tong packet da gui qua DMA */
extern volatile uint32_t g_dbg_tx_busy_skip;  /* so lan DMA ban, phai hoan gui */
extern volatile uint32_t g_dbg_tim6_tick;     /* so lan TIM6 ngat */
extern volatile uint8_t  g_dbg_sensor_data[REAR_N_LIDAR]; /* 1=sensor co data */
extern volatile uint32_t g_dbg_uart_err;      /* so lan UART ErrorCallback */


/* ============================================================
 * PUBLIC API
 * ============================================================ */
void RearLidar_Init(void);
void RearLidar_UART_RxCallback(UART_HandleTypeDef* huart);
void RearLidar_TIM_Callback(TIM_HandleTypeDef* htim);
void RearLidar_TxDone(void);
void RearLidar_UART_ErrorCallback(UART_HandleTypeDef* huart);

#endif /* REAR_LIDAR_STM32_H */