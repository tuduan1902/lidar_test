/**
 * front_lidar_stm32.c
 * ============================================================
 * Xu ly 5 LiDAR VB22A cum dau xe (4 thang + 1 nghieng F4).
 * STM32 KHONG tinh toan o ga - chi doc VB22A, dong goi, gui Jetson.
 * ============================================================
 */
#include "front_lidar_stm32.h"
#include <string.h>

static FrontLidarState s_lidar[FRONT_N_LIDAR];
static UART_HandleTypeDef* const HUART_LIST[FRONT_N_LIDAR] = FRONT_LIDAR_UART_LIST;
static uint8_t s_rx_byte[FRONT_N_LIDAR];

static int find_id(UART_HandleTypeDef* huart) {
    for (int i = 0; i < FRONT_N_LIDAR; i++)
        if (HUART_LIST[i] == huart) return i;
    return -1;
}

/* Checksum VB22A: ~(byte1 + byte2) & 0xFF */
static bool verify_checksum(const uint8_t* pkt) {
    return (uint8_t)(~((uint8_t)(pkt[1] + pkt[2]))) == pkt[3];
}

/* Dong goi 12 bytes va gui qua USART6 */
static void send_packet(uint8_t id, uint16_t dist_mm) {
    const FrontLidarMount* m = &FRONT_LIDAR_MOUNTS[id];

    int16_t angle_10 = (int16_t)(m->angle_deg * 10.0f);
    int16_t ox_mm    = (int16_t)(m->ox_m * 1000.0f);
    int16_t oy_mm    = (int16_t)(m->oy_m * 1000.0f);

    uint8_t pkt[FRONT_PKT_LEN];
    pkt[0]  = FRONT_PKT_HDR;               /* 0xCC */
    pkt[1]  = id;
    pkt[2]  = (uint8_t)(dist_mm & 0xFF);
    pkt[3]  = (uint8_t)(dist_mm >> 8);
    pkt[4]  = (uint8_t)(angle_10 & 0xFF);
    pkt[5]  = (uint8_t)((angle_10 >> 8) & 0xFF);
    pkt[6]  = (uint8_t)(ox_mm & 0xFF);
    pkt[7]  = (uint8_t)((ox_mm >> 8) & 0xFF);
    pkt[8]  = (uint8_t)(oy_mm & 0xFF);
    pkt[9]  = (uint8_t)((oy_mm >> 8) & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= pkt[i];
    pkt[10] = chk;
    pkt[11] = FRONT_PKT_FTR;               /* 0x55 */

    /* 12 bytes @ 460800 bps ~ 208us, ok trong loop 5ms */
    HAL_UART_Transmit(&huart6, pkt, FRONT_PKT_LEN, 2);
}

void FrontLidar_Init(void) {
    memset(s_lidar, 0, sizeof(s_lidar));
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        /* Gui lenh Start Ranging cho tung sensor */
        HAL_UART_Transmit(HUART_LIST[i],
                          (uint8_t*)VB22A_CMD_START,
                          VB22A_CMD_LEN, 5);
        /* Bat dau nhan ngat */
        HAL_UART_Receive_IT(HUART_LIST[i], &s_rx_byte[i], 1);
    }
}

void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    uint8_t b = s_rx_byte[id];
    FrontLidarState* st = &s_lidar[id];

    /* May trang thai 4 byte VB22A:
     *   Doi 0x5C -> nhan dist_L -> nhan dist_H -> nhan checksum */
    if (st->pkt_idx == 0) {
        if (b == VB22A_HEADER) { st->pkt[0] = b; st->pkt_idx = 1; }
    } else {
        st->pkt[st->pkt_idx++] = b;
        if (st->pkt_idx == VB22A_PKT_LEN) {
            if (verify_checksum(st->pkt)) {
                uint16_t dist = (uint16_t)st->pkt[1]
                              | ((uint16_t)st->pkt[2] << 8);
                if (dist >= VB22A_DIST_MIN_MM && dist < VB22A_OUT_OF_RANGE) {
                    st->dist_mm = dist;
                    st->ts_ms   = HAL_GetTick();
                    st->valid   = true;
                }
            }
            st->pkt_idx = 0;
        }
    }
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