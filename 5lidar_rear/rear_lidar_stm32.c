/**
 * rear_lidar_stm32.c
 * ============================================================
 * Xu ly 5 LiDAR VB22A gan duoi xe may tren STM32F407VET6.
 *
 * LUONG XU LY:
 *   1. HAL_UART_Receive_IT(&huartX, &byte, 1) bat dau viec nhan ngat
 *      cho moi UART (5 UART LiDAR).
 *   2. HAL_UART_RxCpltCallback -> RearLidar_UART_RxCallback:
 *      Parse tung byte VB22A vao may trang thai:
 *        - Wait HDR0(0x59) -> Wait HDR1(0x59) -> Nhan 7 byte data+chk
 *        - Kiem tra checksum sum(b[0..7])&0xFF == b[8]
 *        - Luu dist_cm, strength, temp_raw, ts_ms, valid=true
 *      Sau do kick lai IT de nhan byte tiep theo.
 *   3. RearLidar_Process() goi tu vong lap chinh (5ms/lan):
 *      Voi moi sensor co valid=true, dong goi 14 bytes (REAR_PKT)
 *      va gui qua USART6 (HAL_UART_Transmit blocking, ok vi packet nho).
 *      Xoa valid sau khi gui.
 *
 * LAP DAT TRONG PROJECT STM32CUBEIDE:
 *   - Enable USART1-USART3, UART4-UART5, USART6 trong CubeMX.
 *   - Toc do cac LiDAR: 115200. Toc do uplink USART6: 460800.
 *   - Enable "Global Interrupt" cho ca 6 UART trong NVIC.
 *   - Trong stm32f4xx_it.c, cac handler USARTx_IRQHandler() phai goi
 *     HAL_UART_IRQHandler(&huartX) nhu binh thuong (CubeMX tu sinh).
 *   - Trong main.c, them:
 *       void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *           RearLidar_UART_RxCallback(huart);
 *       }
 *     va trong vong lap while(1):
 *       RearLidar_Process();
 *       HAL_Delay(5);
 * ============================================================
 */

#include "rear_lidar_stm32.h"
#include <string.h>

/* ============================================================
 * TRANG THAI NOI BO
 * ============================================================ */
static RearLidarState s_lidar[REAR_N_LIDAR];

/* Mang tro UART khop voi thu tu id sensor (xem rear_lidar_stm32.h) */
static UART_HandleTypeDef* const HUART_LIST[REAR_N_LIDAR] = REAR_LIDAR_UART_LIST;

/* Moi UART IT nhan 1 byte tai 1 thoi diem vao vung nay */
static uint8_t s_rx_byte[REAR_N_LIDAR];

/* ============================================================
 * HAM NOI BO
 * ============================================================ */

/* Tim id sensor tu con tro UART */
static int find_id(UART_HandleTypeDef* huart) {
    for (int i = 0; i < REAR_N_LIDAR; i++) {
        if (HUART_LIST[i] == huart) return i;
    }
    return -1;
}

/* Dong goi va gui 1 packet REAR_PKT (14 bytes) qua USART6 */
static void send_packet(uint8_t id, uint16_t dist_cm,
                        uint16_t strength, uint16_t temp_raw,
                        uint32_t ts_ms) {
    (void)ts_ms; /* ts_ms hien tai khong gui (tiet kiem byte, Jetson tu them) */

    /* Lay thong tin vi tri LiDAR */
    const RearLidarMount* m = &REAR_LIDAR_MOUNTS[id];

    /* Chuyen goc sang int16 x10 de gui integer */
    int16_t angle_10 = (int16_t)(m->angle_deg * 10.0f);

    /* Offset ox mm (gioi han -3276.8 .. 3276.7 cm, du cho xe) */
    int16_t ox_mm = (int16_t)(m->ox_m * 1000.0f);

    uint8_t pkt[REAR_PKT_LEN];
    pkt[0]  = REAR_PKT_HDR;          /* 0xBB */
    pkt[1]  = id;
    pkt[2]  = (uint8_t)(dist_cm & 0xFF);
    pkt[3]  = (uint8_t)(dist_cm >> 8);
    pkt[4]  = (uint8_t)(strength & 0xFF);
    pkt[5]  = (uint8_t)(strength >> 8);
    pkt[6]  = (uint8_t)(temp_raw & 0xFF);
    pkt[7]  = (uint8_t)(temp_raw >> 8);
    pkt[8]  = (uint8_t)(angle_10 & 0xFF);
    pkt[9]  = (uint8_t)((angle_10 >> 8) & 0xFF);
    pkt[10] = (uint8_t)(ox_mm & 0xFF);
    pkt[11] = (uint8_t)((ox_mm >> 8) & 0xFF);

    /* Checksum XOR byte[1..11] */
    uint8_t chk = 0;
    for (int i = 1; i <= 11; i++) chk ^= pkt[i];
    pkt[12] = chk;
    pkt[13] = REAR_PKT_FTR;          /* 0x55 */

    /* Gui blocking (14 bytes @ 460800 ~ 242us, chap nhan duoc trong loop 5ms) */
    HAL_UART_Transmit(&huart6, pkt, REAR_PKT_LEN, 2 /* timeout ms */);
}

/* ============================================================
 * GIAO DIEN PUBLIC
 * ============================================================ */

void RearLidar_Init(void) {
    memset(s_lidar, 0, sizeof(s_lidar));

    /* Bat dau nhan IT cho moi UART LiDAR (1 byte moi lan) */
    for (int i = 0; i < REAR_N_LIDAR; i++) {
        HAL_UART_Receive_IT(HUART_LIST[i], &s_rx_byte[i], 1);
    }
}

/* Goi tu HAL_UART_RxCpltCallback trong main.c */
void RearLidar_UART_RxCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    uint8_t b = s_rx_byte[id];
    RearLidarState* st = &s_lidar[id];

    /* --------------------------------------------------------
     * May trang thai parse VB22A:
     *   HDR0 -> HDR1 -> 7 byte (dist_L,dist_H,str_L,str_H,
     *                            tmp_L,tmp_H,checksum)
     * Packet du: 0x59 0x59 dL dH sL sH tL tH chk
     *            [0]  [1]  [2][3][4][5][6][7][8]
     * -------------------------------------------------------- */
    if (!st->synced) {
        /* Tim header byte dau tien */
        if (b == VB22A_HDR0) {
            st->pkt[0] = b;
            st->pkt_idx = 1;
            st->synced = true; /* co the la dau header */
        }
    } else {
        st->pkt[st->pkt_idx++] = b;

        if (st->pkt_idx == 2) {
            /* Kiem tra header byte thu 2 */
            if (b != VB22A_HDR1) {
                /* Khong phai header, reset */
                st->synced = false;
                st->pkt_idx = 0;
                /* Nhung neu b == HDR0, co the la byte dau cua packet moi */
                if (b == VB22A_HDR0) {
                    st->pkt[0] = b;
                    st->pkt_idx = 1;
                    st->synced = true;
                }
            }
        } else if (st->pkt_idx == VB22A_PKT_LEN) {
            /* Nhan du 9 byte - kiem tra checksum */
            uint8_t chk = 0;
            for (int i = 0; i < 8; i++) chk += st->pkt[i];
            chk &= 0xFF;

            if (chk == st->pkt[8]) {
                /* Packet hop le - parse */
                uint16_t dist_cm   = (uint16_t)st->pkt[2] | ((uint16_t)st->pkt[3] << 8);
                uint16_t strength  = (uint16_t)st->pkt[4] | ((uint16_t)st->pkt[5] << 8);
                uint16_t temp_raw  = (uint16_t)st->pkt[6] | ((uint16_t)st->pkt[7] << 8);

                /* Loc nhieu */
                if (dist_cm >= VB22A_DIST_MIN_CM &&
                    dist_cm <= VB22A_DIST_MAX_CM &&
                    strength >= VB22A_STRENGTH_MIN) {
                    st->dist_cm  = dist_cm;
                    st->strength = strength;
                    st->temp_raw = temp_raw;
                    st->ts_ms    = HAL_GetTick();
                    st->valid    = true;
                }
            }

            /* Reset cho packet tiep theo */
            st->synced  = false;
            st->pkt_idx = 0;
        }
    }

    /* Kick lai IT nhan byte ke tiep */
    HAL_UART_Receive_IT(huart, &s_rx_byte[id], 1);
}

void RearLidar_Process(void) {
    for (int i = 0; i < REAR_N_LIDAR; i++) {
        RearLidarState* st = &s_lidar[i];
        if (!st->valid) continue;

        /* Snapshot gia tri (tranh race voi callback ngat) */
        uint16_t dist_cm  = st->dist_cm;
        uint16_t strength = st->strength;
        uint16_t temp_raw = st->temp_raw;
        uint32_t ts_ms    = st->ts_ms;
        st->valid = false; /* xoa truoc khi gui de khong gui lai */

        send_packet((uint8_t)i, dist_cm, strength, temp_raw, ts_ms);
    }
}