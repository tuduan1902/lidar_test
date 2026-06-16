/**
 * front_lidar_stm32.c
 * ============================================================
 * Xu ly 4 LiDAR VB22A dau xe may tren STM32F407VET6.
 *
 * PROTOCOL VB22A DUNG (4 bytes, doc tu datasheet V3.0):
 *   [0] 0x5C  header
 *   [1] dist_L
 *   [2] dist_H  -> dist_mm = dist_H<<8 | dist_L (don vi MM)
 *   [3] checksum = ~(dist_L + dist_H) & 0xFF
 *
 * LUONG XU LY:
 *   Init:    Gui lenh "Start Ranging" cho moi LiDAR qua TX.
 *   Nhan:    HAL_UART_Receive_IT (1 byte/lan) -> callback ->
 *            may trang thai 4 buoc -> kiem tra checksum ->
 *            luu dist_mm, valid=true.
 *   Process: Moi 5ms, quet 4 sensor, neu valid -> dong goi
 *            12 bytes -> gui USART6 -> Jetson.
 * ============================================================
 */

#include "front_lidar_stm32.h"
#include <string.h>

/* ============================================================
 * TRANG THAI NOI BO
 * ============================================================ */
static FrontLidarState s_lidar[FRONT_N_LIDAR];
static UART_HandleTypeDef* const HUART_LIST[FRONT_N_LIDAR] = FRONT_LIDAR_UART_LIST;
static uint8_t s_rx_byte[FRONT_N_LIDAR];

/* ============================================================
 * HAM NOI BO
 * ============================================================ */
static int find_id(UART_HandleTypeDef* huart) {
    for (int i = 0; i < FRONT_N_LIDAR; i++)
        if (HUART_LIST[i] == huart) return i;
    return -1;
}

/* Kiem tra checksum VB22A: ~(byte1 + byte2) & 0xFF */
static bool verify_checksum(const uint8_t* pkt) {
    uint8_t expected = (uint8_t)(~((uint8_t)(pkt[1] + pkt[2])));
    return expected == pkt[3];
}

/* Dong goi va gui 12 bytes uplink len Jetson qua USART6 */
static void send_packet(uint8_t id, uint16_t dist_mm) {
    const FrontLidarMount* m = &FRONT_LIDAR_MOUNTS[id];

    int16_t angle_10 = (int16_t)(m->angle_deg * 10.0f); /* 900 */
    int16_t ox_mm    = (int16_t)(m->ox_m * 1000.0f);
    int16_t oy_mm    = (int16_t)(m->oy_m * 1000.0f);

    uint8_t pkt[FRONT_PKT_LEN];
    pkt[0]  = FRONT_PKT_HDR;                      /* 0xCC */
    pkt[1]  = id;
    pkt[2]  = (uint8_t)(dist_mm & 0xFF);
    pkt[3]  = (uint8_t)(dist_mm >> 8);
    pkt[4]  = (uint8_t)(angle_10 & 0xFF);
    pkt[5]  = (uint8_t)((angle_10 >> 8) & 0xFF);
    pkt[6]  = (uint8_t)(ox_mm & 0xFF);
    pkt[7]  = (uint8_t)((ox_mm >> 8) & 0xFF);
    pkt[8]  = (uint8_t)(oy_mm & 0xFF);
    pkt[9]  = (uint8_t)((oy_mm >> 8) & 0xFF);

    /* Checksum XOR byte[1..9] */
    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= pkt[i];
    pkt[10] = chk;
    pkt[11] = FRONT_PKT_FTR;                      /* 0x55 */

    /* Blocking TX: 12 bytes @ 460800 ~ 208us, ok trong loop 5ms */
    HAL_UART_Transmit(&huart6, pkt, FRONT_PKT_LEN, 2);
}

/* ============================================================
 * GIAO DIEN PUBLIC
 * ============================================================ */
void FrontLidar_Init(void) {
    memset(s_lidar, 0, sizeof(s_lidar));

    /* Gui lenh Start Ranging va kick IT nhan cho moi sensor */
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        /* TX lenh start (blocking ngan, 6 bytes ~ 100us) */
        HAL_UART_Transmit(HUART_LIST[i],
                          (uint8_t*)VB22A_CMD_START,
                          VB22A_CMD_LEN, 5);
        /* Bat dau nhan IT */
        HAL_UART_Receive_IT(HUART_LIST[i], &s_rx_byte[i], 1);
    }
}

void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    uint8_t b = s_rx_byte[id];
    FrontLidarState* st = &s_lidar[id];

    if (st->pkt_idx == 0) {
        /* Cho header 0x5C */
        if (b == VB22A_HEADER) {
            st->pkt[0] = b;
            st->pkt_idx = 1;
        }
    } else {
        /* Nhan 3 byte con lai: dist_L, dist_H, checksum */
        st->pkt[st->pkt_idx++] = b;

        if (st->pkt_idx == VB22A_PKT_LEN) {
            /* Du 4 byte - kiem tra checksum */
            if (verify_checksum(st->pkt)) {
                uint16_t dist = (uint16_t)st->pkt[1]
                              | ((uint16_t)st->pkt[2] << 8);

                /* Loc: bo qua blind zone va out-of-range */
                if (dist >= VB22A_DIST_MIN_MM && dist < VB22A_OUT_OF_RANGE) {
                    st->dist_mm = dist;
                    st->ts_ms   = HAL_GetTick();
                    st->valid   = true;
                }
            }
            /* Reset cho frame tiep theo */
            st->pkt_idx = 0;
        }
    }

    /* Kick lai IT */
    HAL_UART_Receive_IT(huart, &s_rx_byte[id], 1);
}

void FrontLidar_Process(void) {
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        FrontLidarState* st = &s_lidar[i];
        if (!st->valid) continue;

        uint16_t dist = st->dist_mm;
        st->valid = false;

        send_packet((uint8_t)i, dist);
    }
}