/**
 * front_lidar_stm32.h
 * ============================================================
 * CAU HINH 4 LiDAR VB22A gan dau xe may, tat ca huong THANG RA TRUOC
 *
 * PROTOCOL THUC TE VB22A V3.0 (doc tu datasheet chinh thuc):
 * -------------------------------------------------------
 *   Baudrate mac dinh: 460800
 *   OUTPUT: 4 BYTES moi frame
 *     [0] = 0x5C          header co dinh
 *     [1] = dist_L        khoang cach byte thap  (don vi MM)
 *     [2] = dist_H        khoang cach byte cao   (don vi MM)
 *     [3] = checksum      = ~(dist_L + dist_H) & 0xFF (dao bit tong)
 *   dist_mm = dist_H<<8 | dist_L, range 0-20000 mm
 *   Out-of-range: sensor tra ve 20000
 *
 *   LENH KHOI DONG (phai gui truoc khi nhan data):
 *     Start Ranging: 5A 0A 02 02 00 F1  (6 bytes)
 *     Stop  Ranging: 5A 0A 02 00 00 F3
 *
 * SAI TRONG CODE CU (da sua lai):
 *   - Cu: 9 bytes, header 0x59 0x59, checksum XOR, don vi cm
 *   - Moi: 4 bytes, header 0x5C,  checksum ~sum,  don vi mm
 *
 * SO DO BO TRI (nhin tu tren xuong, mui xe huong len tren):
 *
 *    ↑ ↑   ↑ ↑       <- 4 tia, tat ca thang truoc (angle=90 do)
 *    | |   | |
 *  [F0][F1][F2][F3]  <- dau xe
 *    |  |   |  |
 *  ngoai trong trong ngoai
 *  trai  trai phai  phai
 *
 *   F0: ngoai trai,  sat banh truoc  (ox=-0.30m, oy=+0.90m)
 *   F1: trong trai,  lui vo bung xe  (ox=-0.15m, oy=+0.60m)
 *   F2: trong phai,  lui vo bung xe  (ox=+0.15m, oy=+0.60m)
 *   F3: ngoai phai,  sat banh truoc  (ox=+0.30m, oy=+0.90m)
 *
 * TINH TOAN TOA DO DIEM TREN MAP (angle=90 do cho tat ca):
 *   wx = ox + dist_m * cos(90°) = ox + 0     = ox
 *   wy = oy + dist_m * sin(90°) = oy + dist_m
 *   => Diem nam thang phia truoc moi sensor, chinh xac.
 *
 * DIEU CHINH VI TRI THUC TE:
 *   Chi sua ox_m va oy_m trong FRONT_LIDAR_MOUNTS[].
 *   angle_deg = 90.0f cho tat ca - KHONG sua tru khi lap cham goc.
 *
 * HARDWARE STM32F407VET6:
 *   F0 -> USART1 (PA9=TX, PA10=RX)  @ 460800
 *   F1 -> USART2 (PA2=TX, PA3=RX)   @ 460800
 *   F2 -> USART3 (PB10=TX, PB11=RX) @ 460800
 *   F3 -> UART4  (PC10=TX, PC11=RX) @ 460800
 *   Uplink -> USART6 (PC6=TX, PC7=RX) @ 460800 -> USB-UART -> /dev/ttyUSB3
 *
 * TICH HOP VAO PROJECT CUBEMX:
 *   - Enable USART1/2/3, UART4, USART6 trong CubeMX, baudrate 460800.
 *   - Enable Global Interrupt cho ca 5 UART trong NVIC.
 *   - Them vao main.c:
 *       void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *           FrontLidar_UART_RxCallback(huart);
 *       }
 *   - Trong while(1):
 *       FrontLidar_Process();
 *       HAL_Delay(5);
 * ============================================================
 */
#ifndef FRONT_LIDAR_STM32_H
#define FRONT_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * PROTOCOL VB22A (4 BYTES)
 * ============================================================ */
#define VB22A_PKT_LEN       4
#define VB22A_HEADER        0x5C

/* Lenh 6 bytes gui xuong sensor */
#define VB22A_CMD_LEN       6
static const uint8_t VB22A_CMD_START[VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x02,0x00,0xF1};
static const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x00,0x00,0xF3};

/* Khoang cach out-of-range theo datasheet */
#define VB22A_OUT_OF_RANGE  20000  /* mm */
#define VB22A_DIST_MIN_MM   50     /* blind zone 5cm = 50mm */
#define VB22A_DIST_MAX_MM   19999  /* <20000 la do duoc */

/* ============================================================
 * PACKET UPLINK STM32 -> JETSON (12 BYTES)
 * [0]    = 0xCC  header (khac rear 0xBB, road 0xAA)
 * [1]    = id    (0-3)
 * [2,3]  = dist_mm  uint16 LE  (don vi MM, thang tu sensor)
 * [4,5]  = angle_10 int16  LE  (goc x10 do, luon = 900)
 * [6,7]  = ox_mm    int16  LE  (offset X mm)
 * [8,9]  = oy_mm    int16  LE  (offset Y mm)
 * [10]   = checksum XOR byte[1..9]
 * [11]   = 0x55  footer
 * ============================================================ */
#define FRONT_PKT_LEN       12
#define FRONT_PKT_HDR       0xCC
#define FRONT_PKT_FTR       0x55

/* ============================================================
 * SO LUONG SENSOR
 * ============================================================ */
#define FRONT_N_LIDAR       4

/* ============================================================
 * CAU HINH VI TRI CAC LIDAR
 * DIEU CHINH ox_m VA oy_m THEO THUC TE LAP DAT
 * angle_deg = 90.0f cho tat ca (thang truoc)
 * ============================================================ */
typedef struct {
    float angle_deg; /* 90.0 = thang truoc */
    float ox_m;      /* offset X (m): am=trai xe, duong=phai xe */
    float oy_m;      /* offset Y (m): duong=phia truoc tam xe   */
} FrontLidarMount;

static const FrontLidarMount FRONT_LIDAR_MOUNTS[FRONT_N_LIDAR] = {
    /* id=0  F0 ngoai trai, sat banh truoc */ { 90.0f, -0.30f, +0.90f },
    /* id=1  F1 trong trai, lui vo bung xe */ { 90.0f, -0.15f, +0.60f },
    /* id=2  F2 trong phai, lui vo bung xe */ { 90.0f, +0.15f, +0.60f },
    /* id=3  F3 ngoai phai, sat banh truoc */ { 90.0f, +0.30f, +0.90f },
};

/* ============================================================
 * UART PERIPHERAL MAPPING
 * ============================================================ */
extern UART_HandleTypeDef huart1; /* F0 */
extern UART_HandleTypeDef huart2; /* F1 */
extern UART_HandleTypeDef huart3; /* F2 */
extern UART_HandleTypeDef huart4; /* F3 */
extern UART_HandleTypeDef huart6; /* Uplink Jetson */

#define FRONT_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4 }

/* ============================================================
 * TRANG THAI MOI SENSOR
 * ============================================================ */
typedef struct {
    uint8_t  pkt[VB22A_PKT_LEN]; /* byte dang tich luy */
    uint8_t  pkt_idx;             /* vi tri dang ghi (0=cho header) */
    uint16_t dist_mm;             /* gia tri do duoc (mm) */
    bool     valid;               /* co du lieu moi chua */
    uint32_t ts_ms;
} FrontLidarState;

/* ============================================================
 * GIAO DIEN PUBLIC
 * ============================================================ */
void FrontLidar_Init(void);    /* Goi 1 lan sau HAL_Init() */
void FrontLidar_Process(void); /* Goi trong while(1) */
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart);

#endif /* FRONT_LIDAR_STM32_H */