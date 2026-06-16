/**
 * rear_lidar_stm32.h
 * ============================================================
 * CAU HINH 5 LiDAR VB22A gan sau duoi xe may, bo tri vong cung 90°
 *
 * SO DO BO TRI (nhin tu tren xuong, mui xe huong len tren):
 *
 *         [mui xe]
 *            ^
 *            |
 *   ---------+--------- tam xe
 *            |
 *   [L2] [L1] [L0] [L3] [L4]   <- duoi xe (phia sau)
 *    \    \    |    /    /
 *     \    \   |   /    /
 *      45°  22.5° |  22.5° 45°
 *
 *   L0: chinh giua, huong thang ra sau  (angle = 180°, +Y = phia truoc xe)
 *   L1: lech trai  22.5°               (angle = 180° - 22.5° = 157.5° tinh tu X+)
 *   L2: lech trai  45°                 (angle = 180° - 45°   = 135°)
 *   L3: lech phai  22.5°               (angle = 180° + 22.5° = 202.5°)
 *   L4: lech phai  45°                 (angle = 180° + 45°   = 225°)
 *
 * Goc duoc dinh nghia theo he toa do XY cua xe:
 *   +Y = phia truoc xe, +X = ben phai xe
 *   Goc do tu truc +X, nguoc chieu kim dong ho
 *   Nen 90° = thang trai, 180° = thang sau, 270° = thang phai
 *
 * Moi LiDAR duoc cap 1 UART rieng biet tren STM32F407VET6:
 *   L0 -> USART1 (PA9=TX, PA10=RX)
 *   L1 -> USART2 (PA2=TX, PA3=RX)
 *   L2 -> USART3 (PB10=TX, PB11=RX)
 *   L3 -> UART4  (PC10=TX, PC11=RX)
 *   L4 -> UART5  (PC12=TX, PD2=RX)
 *
 * Uplink Jetson:
 *   USART6 (PC6=TX, PC7=RX) -> USB-UART converter -> /dev/ttyUSBx tren Jetson
 *   Baudrate: 460800 bps (du bam cho 5 LiDAR x 200Hz x 12 bytes)
 *
 * VB22A protocol:
 *   Baudrate: 115200 bps
 *   Packet: [0x59][0x59][distL][distH][strength_L][strength_H][temp_L][temp_H][checksum]
 *   9 bytes, checksum = sum(byte[0..7]) & 0xFF
 *   dist = (distH << 8 | distL) in cm (!)  -- khac voi VB22A firmware khac
 *
 * DIEU CHINH VI TRI THUC TE:
 *   Sua REAR_LIDAR_MOUNTS[] ben duoi:
 *   - angle_deg: goc cua tia so voi truc +X xe (do, nguoc chieu kim dong ho)
 *   - ox_m / oy_m: offset vi tri dat sensor so voi tam xe (m)
 *     oy_m < 0 = phia sau xe (duoi xe)
 * ============================================================
 */
#ifndef REAR_LIDAR_STM32_H
#define REAR_LIDAR_STM32_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * SO LUONG SENSOR VA KICH THUOC PACKET
 * ============================================================ */
#define REAR_N_LIDAR        5
#define VB22A_PKT_LEN       9       /* bytes moi packet VB22A */
#define VB22A_HDR0          0x59    /* byte header thu nhat */
#define VB22A_HDR1          0x59    /* byte header thu hai */

/* ============================================================
 * PACKET UPLINK (STM32 -> Jetson), 14 bytes
 * [0]    = 0xBB  (header, khac 0xAA de phan biet voi road_scanner)
 * [1]    = id    (0-4, chi so LiDAR)
 * [2,3]  = dist_cm (uint16, little-endian, don vi cm)
 * [4,5]  = strength (uint16, little-endian, tin hieu phan xa)
 * [6,7]  = temp_raw (uint16, little-endian, nhiet do cam bien x10 do C)
 * [8,9]  = angle_10 (int16, little-endian, goc x10 do, vd 1800=180.0 do)
 * [10,11]= ox_mm (int16, little-endian, offset X mm)
 * [12]   = checksum XOR byte[1..11]
 * [13]   = 0x55  (footer)
 * ============================================================ */
#define REAR_PKT_LEN        14
#define REAR_PKT_HDR        0xBB
#define REAR_PKT_FTR        0x55

/* ============================================================
 * CAU HINH VI TRI CAC LIDAR (DIEU CHINH THEO THUC TE)
 * ============================================================ */
typedef struct {
    float angle_deg;  /* goc tia so voi truc +X xe (deg, CCW duong) */
    float ox_m;       /* offset X (m), + = phai xe */
    float oy_m;       /* offset Y (m), + = truoc xe, < 0 = phia sau xe */
} RearLidarMount;

/* ============================================================
 * THAY DOI BANG NAY DE KHOP VOI THUC TE LAP DAT
 *
 * Mac dinh: xe rong ~0.6m, 5 sensor trai deu 15cm
 * oy_m = -0.85m: LiDAR dat 85cm phia sau tam xe (duoi xe may)
 *
 *        ox_m:  -0.30 -0.15  0.00 +0.15 +0.30
 * sensor id:      L2    L1    L0    L3    L4
 * ============================================================ */
static const RearLidarMount REAR_LIDAR_MOUNTS[REAR_N_LIDAR] = {
    /* id=0 */ { 180.0f,  0.00f, -0.85f }, /* trung tam, thang ra sau */
    /* id=1 */ { 157.5f, -0.15f, -0.85f }, /* lech trai 22.5 do */
    /* id=2 */ { 135.0f, -0.30f, -0.85f }, /* lech trai 45 do */
    /* id=3 */ { 202.5f,  0.15f, -0.85f }, /* lech phai 22.5 do */
    /* id=4 */ { 225.0f,  0.30f, -0.85f }, /* lech phai 45 do */
};

/* ============================================================
 * MAPPING UART -> PERIPHERAL HAL
 * Khop voi pin assignment o dau file
 * ============================================================ */
/* Khai bao extern - dinh nghia thuc su trong main.c cua project STM32 */
extern UART_HandleTypeDef huart1; /* L0 */
extern UART_HandleTypeDef huart2; /* L1 */
extern UART_HandleTypeDef huart3; /* L2 */
extern UART_HandleTypeDef huart4; /* L3 */
extern UART_HandleTypeDef huart5; /* L4 */
extern UART_HandleTypeDef huart6; /* Uplink -> Jetson */

/* Mang de tro nguoi loop co the dung [id] truc tiep */
#define REAR_LIDAR_UART_LIST { &huart1, &huart2, &huart3, &huart4, &huart5 }

/* ============================================================
 * NGUONG LOC
 * ============================================================ */
#define VB22A_DIST_MIN_CM    20    /* bo qua do nho hon 20cm (nhieu gan) */
#define VB22A_DIST_MAX_CM    800   /* bo qua qua xa 8m */
#define VB22A_STRENGTH_MIN   100   /* tin hieu phan xa qua yeu = nhieu */

/* ============================================================
 * TRANG THAI MOI SENSOR (dung trong rear_lidar_stm32.c)
 * ============================================================ */
typedef struct {
    uint8_t  rxbuf[VB22A_PKT_LEN * 4]; /* vung dem DMA / IT */
    uint8_t  pkt[VB22A_PKT_LEN];       /* packet dang parse */
    uint8_t  pkt_idx;                   /* vi tri dang ghi */
    bool     synced;                    /* da tim thay header chua */

    uint16_t dist_cm;
    uint16_t strength;
    uint16_t temp_raw;
    bool     valid;
    uint32_t ts_ms;
} RearLidarState;

/* ============================================================
 * GIAO DIEN PUBLIC
 * ============================================================ */
/* Goi 1 lan trong main() sau khi HAL_Init() va MX_UARTx_Init() */
void RearLidar_Init(void);

/* Goi trong vong lap chinh (hoac timer callback 5ms).
 * Kiem tra du lieu moi tu moi UART, neu co packet hop le thi
 * dong goi va gui len Jetson qua USART6. */
void RearLidar_Process(void);

/* Callback HAL - goi trong HAL_UART_RxCpltCallback() cua main.c */
void RearLidar_UART_RxCallback(UART_HandleTypeDef* huart);

#endif /* REAR_LIDAR_STM32_H */