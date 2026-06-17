/**
 * front_lidar_stm32.c
 * ============================================================
 * - RX: UART IT, parse VB22A 4 bytes theo header 0x5C
 * - TX: USART6 DMA, CPU khong bi block khi gui packet Jetson
 * - Trigger: TIM6 ngat 5ms thay cho HAL_Delay trong while(1)
 * ============================================================
 */
#include "front_lidar_stm32.h"
#include <string.h>

/* ============================================================
 * TRANG THAI NOI BO
 * ============================================================ */
static FrontLidarState s_lidar[FRONT_N_LIDAR];
static UART_HandleTypeDef* const HUART_LIST[FRONT_N_LIDAR] = FRONT_LIDAR_UART_LIST;
static uint8_t s_rx_byte[FRONT_N_LIDAR]; /* moi UART IT nhan 1 byte */

/* Buffer DMA TX: 1 packet tai 1 thoi diem, kich thuoc = 12 bytes.
 * Khai bao __attribute__((aligned(4))) de dam bao alignment cho DMA. */
static uint8_t s_tx_buf[FRONT_PKT_LEN] __attribute__((aligned(4)));

/* Co tranh chong cheo DMA TX: neu dang truyen thi bo qua packet moi.
 * Chap nhan mat 1 packet hon la corrupt DMA. */
static volatile bool s_tx_busy = false;

/* ============================================================
 * HAM NOI BO
 * ============================================================ */
static int find_id(UART_HandleTypeDef* huart) {
    for (int i = 0; i < FRONT_N_LIDAR; i++)
        if (HUART_LIST[i] == huart) return i;
    return -1;
}

static bool verify_checksum(const uint8_t* pkt) {
    /* VB22A: checksum = ~(byte1 + byte2) & 0xFF */
    return (uint8_t)(~((uint8_t)(pkt[1] + pkt[2]))) == pkt[3];
}

static void send_packet_dma(uint8_t id, uint16_t dist_mm) {
    /* Neu DMA dang ban thi bo qua, tranh ghi de buffer */
    if (s_tx_busy) return;

    const FrontLidarMount* m = &FRONT_LIDAR_MOUNTS[id];
    int16_t angle_10 = (int16_t)(m->angle_deg * 10.0f);
    int16_t ox_mm    = (int16_t)(m->ox_m * 1000.0f);
    int16_t oy_mm    = (int16_t)(m->oy_m * 1000.0f);

    s_tx_buf[0]  = FRONT_PKT_HDR;
    s_tx_buf[1]  = id;
    s_tx_buf[2]  = (uint8_t)(dist_mm & 0xFF);
    s_tx_buf[3]  = (uint8_t)(dist_mm >> 8);
    s_tx_buf[4]  = (uint8_t)(angle_10 & 0xFF);
    s_tx_buf[5]  = (uint8_t)((angle_10 >> 8) & 0xFF);
    s_tx_buf[6]  = (uint8_t)(ox_mm & 0xFF);
    s_tx_buf[7]  = (uint8_t)((ox_mm >> 8) & 0xFF);
    s_tx_buf[8]  = (uint8_t)(oy_mm & 0xFF);
    s_tx_buf[9]  = (uint8_t)((oy_mm >> 8) & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= s_tx_buf[i];
    s_tx_buf[10] = chk;
    s_tx_buf[11] = FRONT_PKT_FTR;

    s_tx_busy = true;
    /* DMA TX: CPU tra ve ngay lap tuc, DMA tu dong gui 12 bytes.
     * Khi xong, HAL goi HAL_UART_TxCpltCallback -> s_tx_busy = false */
    HAL_UART_Transmit_DMA(&huart6, s_tx_buf, FRONT_PKT_LEN);
}

/* ============================================================
 * PUBLIC: INIT
 * Goi trong USER CODE BEGIN 2 cua main.c
 * ============================================================ */
void FrontLidar_Init(void) {
    memset(s_lidar, 0, sizeof(s_lidar));
    s_tx_busy = false;

    /* Gui lenh Start Ranging cho tung sensor (blocking nho, chi chay 1 lan) */
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        HAL_UART_Transmit(HUART_LIST[i],
                          (uint8_t*)VB22A_CMD_START,
                          VB22A_CMD_LEN, 10);
        /* Kick IT nhan byte dau tien */
        HAL_UART_Receive_IT(HUART_LIST[i], &s_rx_byte[i], 1);
    }

    /* Bat dau TIM6: ngat moi 5ms se goi FrontLidar_TIM_Callback */
    HAL_TIM_Base_Start_IT(&htim6);
}

/* ============================================================
 * PUBLIC: UART IT CALLBACK
 * Goi trong HAL_UART_RxCpltCallback cua main.c
 * Chay trong ISR - giu ngan, khong goi HAL_Delay
 * ============================================================ */
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    uint8_t b = s_rx_byte[id];
    FrontLidarState* st = &s_lidar[id];

    /* May trang thai 4 byte VB22A */
    if (st->pkt_idx == 0) {
        if (b == VB22A_HEADER) {
            st->pkt[0] = b;
            st->pkt_idx = 1;
        }
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

    /* Kick lai IT nhan byte tiep theo */
    HAL_UART_Receive_IT(huart, &s_rx_byte[id], 1);
}

/* ============================================================
 * PUBLIC: TIM6 CALLBACK (chay trong ISR, moi 5ms)
 * Goi trong HAL_TIM_PeriodElapsedCallback cua main.c
 * Thay the hoan toan cho HAL_Delay(5) + FrontLidar_Process()
 * trong while(1) cu.
 * ============================================================ */
void FrontLidar_TIM_Callback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;

    /* Quet 5 sensor, gui packet DMA neu co data moi */
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        FrontLidarState* st = &s_lidar[i];
        if (!st->valid) continue;
        uint16_t dist = st->dist_mm;
        st->valid = false;          /* xoa truoc khi gui */
        send_packet_dma((uint8_t)i, dist);

        /* Chi gui 1 packet moi ISR de khong chen co s_tx_busy.
         * Cac sensor con lai se duoc xu ly o lan ngat ke tiep (5ms sau).
         * VB22A 200Hz = 5ms/sample, khop dung voi chu ky ngat TIM6. */
        break;
    }
}

/* ============================================================
 * HAL_UART_TxCpltCallback
 * Duoc goi tu DMA TX complete interrupt khi USART6 gui xong.
 * Giai phong co s_tx_busy de cho phep gui packet tiep theo.
 *
 * THEM VAO main.c USER CODE BEGIN 4:
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART6) {
 *           FrontLidar_TxDone();
 *       }
 *   }
 * ============================================================ */
void FrontLidar_TxDone(void) {
    s_tx_busy = false;
}