/**
 * front_lidar_stm32.h
 * ============================================================
 * CUM 5 LiDAR VB22A DAU XE MAY tren STM32F407VET6:
 *   F0-F3: chieu THANG RA TRUOC (angle=90 do)
 *   F4   : LiDAR NGHIENG xuong mat duong (phat hien o ga)
 *
 * SO DO BO TRI (nhin tu tren xuong, mui xe huong len tren):
 *
 *    ↑ ↑  ↑(nghieng)  ↑ ↑
 *    | |      |        | |
 *  [F0][F1] [F4]     [F2][F3]
 *  ngoai trong nghieng trong ngoai
 *  trai  trai  giua    phai  phai
 *
 * F4 - LiDAR nghieng (thay the road_scanner truoc day):
 *   - Goc chui: PITCH_STATIC_RAD = asin(H_MOUNT/SLANT_RANGE_REF)
 *   - STM32 KHONG tinh toan o ga, chi gui dist_mm len Jetson
 *   - Jetson (front_scanner.cpp) se tinh delta va phat hien o ga
 *   - ox=0 (giua xe), oy=+FRONT_TILT_OY_M (vi tri doc)
 *   - angle_deg = FRONT_TILT_ANGLE_DEG (goc nghieng, tinh tu +X)
 *     Gia tri nay >= 90 do vi tia chui xuong phia truoc-duoi
 *     Vi du: goc pitch 30 do so voi ngang -> angle = 90+30 = 120?
 *     Khong! He toa do map dung sin/cos(angle) theo truc XY san:
 *     Goc chui trong he toa do 3D (pitch), khong phai goc 2D map.
 *     -> F4 gui angle=90 (huong toi diem chieu xuong mat duong),
 *        Jetson dung hinh hoc 3D rieng (H_MOUNT, SLANT_RANGE_REF)
 *        de tinh expected_dist va delta, KHONG dung angle de map.
 *        Jetson map diem o ga tai (ox=0, oy_chieu) bang cos(pitch).
 *
 * PROTOCOL VB22A (4 bytes, datasheet V3.0):
 *   [0]=0x5C  [1]=dist_L  [2]=dist_H  [3]=~(dist_L+dist_H)
 *   dist don vi MM, baudrate 460800
 *   Lenh start: 5A 0A 02 02 00 F1
 *
 * PACKET UPLINK 12 bytes (STM32 -> Jetson, header 0xCC):
 *   [0]=0xCC [1]=id(0-4) [2,3]=dist_mm [4,5]=angle_10
 *   [6,7]=ox_mm [8,9]=oy_mm [10]=XOR[1..9] [11]=0x55
 *   F4 gui angle_10=900 (angle=90, phu hieu, Jetson khong dung de map)
 *   F4 gui ox_mm=0, oy_mm=FRONT_TILT_OY_MM
 *
 * HARDWARE:
 *   F0->USART1  F1->USART2  F2->USART3
 *   F3->UART4   F4->UART5   Uplink->USART6@460800
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
#define FRONT_N_LIDAR       5      /* 4 thang + 1 nghieng */
#define FRONT_ID_TILT       4      /* id cua LiDAR nghieng */

/* ============================================================
 * CAU HINH VI TRI (DIEU CHINH THEO THUC TE)
 *
 * F0-F3: angle=90 do (thang truoc), ox/oy theo vi tri lap
 * F4   : LiDAR nghieng - STM32 gui angle=90 va oy thuc te
 *        (Jetson se dung hinh hoc pitch rieng, khong dung angle nay)
 *
 * FRONT_TILT_OY_M : vi tri doc cua F4 so voi tam xe (m)
 *                   Duong = phia truoc. Sua theo thuc te.
 * ============================================================ */
#define FRONT_TILT_OY_MM    800    /* 80cm phia truoc tam xe */

typedef struct {
    float    angle_deg;  /* goc gui len Jetson (F4: 90.0, Jetson dung pitch rieng) */
    float    ox_m;
    float    oy_m;
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
extern UART_HandleTypeDef huart5; /* F4 LiDAR nghieng */
extern UART_HandleTypeDef huart6; /* Uplink Jetson */

#define FRONT_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4, &huart5 }

/* ============================================================
 * TRANG THAI
 * ============================================================ */
typedef struct {
    uint8_t  pkt[VB22A_PKT_LEN];
    uint8_t  pkt_idx;
    uint16_t dist_mm;
    bool     valid;
    uint32_t ts_ms;
} FrontLidarState;

/* ============================================================
 * PUBLIC
 * ============================================================ */
void FrontLidar_Init(void);
void FrontLidar_Process(void);
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart);

#endif /* FRONT_LIDAR_STM32_H */