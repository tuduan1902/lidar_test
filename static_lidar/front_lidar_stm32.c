/**
 * front_lidar_stm32.c  (fixed version)
 * ============================================================
 * BUG DA SUA:
 *   1. TIM6 callback: bo "break", gui TAT CA sensor co data
 *      trong 1 ISR (khong chi sensor dau tien).
 *   2. DMA busy: dung pending queue 5 slot.
 *      Neu DMA dang ban: luu vao queue.
 *      TxDone callback: gui tiep tu queue.
 *   3. LED debug PA6: nhay theo trang thai he thong.
 * ============================================================
 */
#include "front_lidar_stm32.h"
#include <string.h>

/* ============================================================
 * HANG SO VA DU LIEU CAU HINH
 * ============================================================ */
const uint8_t VB22A_CMD_START[VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x02,0x00,0xF1};
const uint8_t VB22A_CMD_STOP [VB22A_CMD_LEN] = {0x5A,0x0A,0x02,0x00,0x00,0xF3};

const FrontLidarMount FRONT_LIDAR_MOUNTS[FRONT_N_LIDAR] = {
    /* id=0 F0 ngoai trai  thang truoc */ { 90.0f, -0.30f, +0.90f },
    /* id=1 F1 trong trai  thang truoc */ { 90.0f, -0.15f, +0.60f },
    /* id=2 F2 trong phai  thang truoc */ { 90.0f, +0.15f, +0.60f },
    /* id=3 F3 ngoai phai  thang truoc */ { 90.0f, +0.30f, +0.90f },
    /* id=4 F4 nghieng quet o ga       */ { 90.0f,  0.00f, +0.80f },
};

/* ============================================================
 * BIEN DEBUG (khai bao thuc su o day, extern trong .h)
 * ============================================================ */
volatile uint32_t g_dbg_rx_bytes     = 0;
volatile uint32_t g_dbg_pkt_ok       = 0;
volatile uint32_t g_dbg_pkt_bad_chk  = 0;
volatile uint32_t g_dbg_tx_sent      = 0;
volatile uint32_t g_dbg_tx_busy_skip = 0;
volatile uint32_t g_dbg_tim6_tick    = 0;
volatile uint8_t  g_dbg_sensor_data[FRONT_N_LIDAR] = {0};
volatile uint32_t g_dbg_uart_err     = 0;

/* ============================================================
 * TRANG THAI NOI BO
 * ============================================================ */
static FrontLidarState s_lidar[FRONT_N_LIDAR];
static UART_HandleTypeDef* const HUART_LIST[FRONT_N_LIDAR] = FRONT_LIDAR_UART_LIST;
static uint8_t s_rx_byte[FRONT_N_LIDAR];

/* Buffer DMA TX - aligned 4 byte cho DMA */
static uint8_t s_tx_buf[FRONT_PKT_LEN] __attribute__((aligned(4)));
static volatile bool s_tx_busy = false;

/* ============================================================
 * PENDING QUEUE: luu packet khi DMA dang ban
 * Moi slot = 12 bytes. FIFO don gian 5 slot (1 per sensor).
 * ============================================================ */
#define TX_QUEUE_SIZE   FRONT_N_LIDAR
static uint8_t  s_queue_buf[TX_QUEUE_SIZE][FRONT_PKT_LEN];
static volatile uint8_t s_queue_head = 0;  /* vi tri ghi tiep theo */
static volatile uint8_t s_queue_tail = 0;  /* vi tri doc tiep theo */
static volatile uint8_t s_queue_cnt  = 0;  /* so slot dang dung */

/* ============================================================
 * DEBUG LED MACROS
 * ============================================================ */
#ifdef FRONT_DEBUG_LED_ENABLE
  #define LED_ON()     HAL_GPIO_WritePin(FRONT_DEBUG_LED_PORT, FRONT_DEBUG_LED_PIN, GPIO_PIN_SET)
  #define LED_OFF()    HAL_GPIO_WritePin(FRONT_DEBUG_LED_PORT, FRONT_DEBUG_LED_PIN, GPIO_PIN_RESET)
  #define LED_TOGGLE() HAL_GPIO_TogglePin(FRONT_DEBUG_LED_PORT, FRONT_DEBUG_LED_PIN)
#else
  #define LED_ON()
  #define LED_OFF()
  #define LED_TOGGLE()
#endif

/* ============================================================
 * HAM NOI BO
 * ============================================================ */
static int find_id(UART_HandleTypeDef* huart) {
    for (int i = 0; i < FRONT_N_LIDAR; i++)
        if (HUART_LIST[i] == huart) return i;
    return -1;
}

static bool verify_checksum(const uint8_t* pkt) {
    return (uint8_t)(~((uint8_t)(pkt[1] + pkt[2]))) == pkt[3];
}

/* Xay dung 12 bytes packet vao buffer chi dinh */
static void build_packet(uint8_t* buf, uint8_t id, uint16_t dist_mm) {
    const FrontLidarMount* m = &FRONT_LIDAR_MOUNTS[id];
    int16_t angle_10 = (int16_t)(m->angle_deg * 10.0f);
    int16_t ox_mm    = (int16_t)(m->ox_m * 1000.0f);
    int16_t oy_mm    = (int16_t)(m->oy_m * 1000.0f);

    buf[0]  = FRONT_PKT_HDR;
    buf[1]  = id;
    buf[2]  = (uint8_t)(dist_mm & 0xFF);
    buf[3]  = (uint8_t)(dist_mm >> 8);
    buf[4]  = (uint8_t)(angle_10 & 0xFF);
    buf[5]  = (uint8_t)((angle_10 >> 8) & 0xFF);
    buf[6]  = (uint8_t)(ox_mm & 0xFF);
    buf[7]  = (uint8_t)((ox_mm >> 8) & 0xFF);
    buf[8]  = (uint8_t)(oy_mm & 0xFF);
    buf[9]  = (uint8_t)((oy_mm >> 8) & 0xFF);
    uint8_t chk = 0;
    for (int i = 1; i <= 9; i++) chk ^= buf[i];
    buf[10] = chk;
    buf[11] = FRONT_PKT_FTR;
}

/* Bat dau gui DMA tu s_tx_buf.
 * Chi goi khi s_tx_busy = false va s_tx_buf da duoc dien. */
static void start_dma_tx(void) {
    s_tx_busy = true;
    HAL_UART_Transmit_DMA(&huart6, s_tx_buf, FRONT_PKT_LEN);
    g_dbg_tx_sent++;
}

/* Dua packet vao queue.
 * Neu queue day (tat ca slot dang cho): bo qua packet cu nhat (overwrite head). */
static void queue_push(uint8_t id, uint16_t dist_mm) {
    if (s_queue_cnt < TX_QUEUE_SIZE) {
        build_packet(s_queue_buf[s_queue_head], id, dist_mm);
        s_queue_head = (s_queue_head + 1) % TX_QUEUE_SIZE;
        s_queue_cnt++;
    }
    /* Neu day: bo qua, tran bo dem la chap nhan duoc hon la corrupt DMA */
}

/* Lay 1 packet tu queue va gui DMA.
 * Tra ve true neu da kick duoc DMA. */
static bool queue_flush_one(void) {
    if (s_queue_cnt == 0 || s_tx_busy) return false;
    memcpy(s_tx_buf, s_queue_buf[s_queue_tail], FRONT_PKT_LEN);
    s_queue_tail = (s_queue_tail + 1) % TX_QUEUE_SIZE;
    s_queue_cnt--;
    start_dma_tx();
    return true;
}

/* ============================================================
 * PUBLIC: INIT
 * ============================================================ */
void FrontLidar_Init(void) {
    memset(s_lidar, 0, sizeof(s_lidar));
    s_tx_busy    = false;
    s_queue_head = 0;
    s_queue_tail = 0;
    s_queue_cnt  = 0;

    /* PA6 GPIO_Output phai duoc enable trong MX_GPIO_Init() cua main.c
     * (CubeMX: PA6 -> GPIO_Output, label: LED_DEBUG) */
    LED_OFF();

    /* Gui lenh Start Ranging den 5 sensor.
     * Blocking OK o day vi chi chay 1 lan luc khoi dong.
     * Moi sensor: 6 bytes @ 460800 = ~104us -> 5 sensor = ~520us tong */
    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        HAL_UART_Transmit(HUART_LIST[i],
                          (uint8_t*)VB22A_CMD_START,
                          VB22A_CMD_LEN, 10);
        /* Kick IT nhan byte dau tien tu sensor */
        HAL_UART_Receive_IT(HUART_LIST[i], &s_rx_byte[i], 1);
    }

    /* Bat dau TIM6 ngat 5ms */
    HAL_TIM_Base_Start_IT(&htim6);

    /* LED sang 1 giay de bao Init xong */
    LED_ON();
    HAL_Delay(1000);  /* delay 1 lan duy nhat luc init, chap nhan duoc */
    LED_OFF();
}

/* ============================================================
 * PUBLIC: UART RX IT CALLBACK (chay trong ISR)
 * ============================================================ */
void FrontLidar_UART_RxCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    uint8_t b = s_rx_byte[id];
    FrontLidarState* st = &s_lidar[id];
    g_dbg_rx_bytes++;

    /* May trang thai parse 4 byte VB22A:
     * State 0: cho header 0x5C
     * State 1-3: nhan dist_L, dist_H, checksum */
    if (st->pkt_idx == 0) {
        if (b == VB22A_HEADER) {
            st->pkt[0] = b;
            st->pkt_idx = 1;
        }
    } else {
        st->pkt[st->pkt_idx++] = b;

        if (st->pkt_idx == VB22A_PKT_LEN) {
            /* Du 4 byte - kiem tra checksum */
            if (verify_checksum(st->pkt)) {
                uint16_t dist = (uint16_t)st->pkt[1]
                              | ((uint16_t)st->pkt[2] << 8);
                if (dist >= VB22A_DIST_MIN_MM && dist < VB22A_OUT_OF_RANGE) {
                    st->dist_mm = dist;
                    st->ts_ms   = HAL_GetTick();
                    st->valid   = true;
                    g_dbg_pkt_ok++;
                    g_dbg_sensor_data[id] = 1;
                }
                /* dist out of range: packet hop le nhung bo qua */
            } else {
                g_dbg_pkt_bad_chk++;
                /* Neu checksum sai nhieu (g_dbg_pkt_bad_chk lon):
                 * Kiem tra wiring, baudrate, hoac VB22A firmware version */
            }
            st->pkt_idx = 0;
        }
    }

    /* Kick lai IT nhan byte tiep theo - PHAI co, neu khong UART dung nhan */
    HAL_UART_Receive_IT(huart, &s_rx_byte[id], 1);
}

/* ============================================================
 * PUBLIC: TIM6 CALLBACK (ISR, moi 5ms)
 *
 * FIX BUG CHINH: BO "break", gio xu ly TAT CA sensor co data.
 * Logic gui:
 *   - Neu DMA ranh: gui ngay vao DMA, va flush queue neu con data.
 *   - Neu DMA ban: push vao queue, TxDone se flush sau.
 *
 * LED debug: nhay moi 5ms neu co it nhat 1 sensor co data,
 *            nhay moi 500ms (1/100 tick) neu khong co data nao.
 * ============================================================ */
void FrontLidar_TIM_Callback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;

    g_dbg_tim6_tick++;

    bool any_data = false;

    for (int i = 0; i < FRONT_N_LIDAR; i++) {
        FrontLidarState* st = &s_lidar[i];
        if (!st->valid) continue;

        any_data = true;
        uint16_t dist = st->dist_mm;
        st->valid = false;  /* xoa truoc khi xu ly, tranh race condition */

        if (!s_tx_busy && s_queue_cnt == 0) {
            /* DMA ranh va queue trong: gui truc tiep */
            build_packet(s_tx_buf, (uint8_t)i, dist);
            start_dma_tx();
        } else {
            /* DMA ban hoac con data trong queue: giu lai cho TxDone */
            queue_push((uint8_t)i, dist);
            g_dbg_tx_busy_skip++;
        }
        /* KHONG break - tiep tuc quet sensor ke tiep */
    }

    /* LED debug: phan anh trang thai he thong
     * Co data     -> nhay moi 5ms (nhan ra ngay khi nhin vao board)
     * Khong data  -> nhay cham 500ms (lo sensor chua gui data)
     * DMA stuck   -> sang lien tuc (s_tx_busy khong duoc giai phong) */
    if (s_tx_busy && g_dbg_tim6_tick > 200 && s_queue_cnt >= TX_QUEUE_SIZE) {
        /* DMA busy qua 1 giay va queue day: co the TxDone callback khong chay */
        LED_ON();  /* sang lien tuc = bao loi */
    } else if (any_data) {
        LED_TOGGLE();  /* nhay nhanh = co data */
    } else {
        /* Nhay cham: chi toggle moi 100 tick = 500ms */
        if ((g_dbg_tim6_tick % 100) == 0) {
            LED_TOGGLE();
        }
    }
}

/* ============================================================
 * PUBLIC: DMA TX COMPLETE (goi tu HAL_UART_TxCpltCallback)
 * ============================================================ */
void FrontLidar_TxDone(void) {
    s_tx_busy = false;
    /* Ngay lap tuc flush packet tiep theo trong queue neu co */
    queue_flush_one();
}

/* ============================================================
 * PUBLIC: UART ERROR CALLBACK
 * Phai xu ly de khong bi treo khi co overrun/framing error
 * ============================================================ */
void FrontLidar_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    int id = find_id(huart);
    if (id < 0) return;

    g_dbg_uart_err++;

    /* Xoa co loi - quan trong: neu khong xoa, UART se tiep tuc bao loi */
    __HAL_UART_CLEAR_OREFLAG(huart);  /* Overrun */
    __HAL_UART_CLEAR_NEFLAG(huart);   /* Noise */
    __HAL_UART_CLEAR_FEFLAG(huart);   /* Framing */

    /* Reset state machine cua sensor bi loi */
    s_lidar[id].pkt_idx = 0;

    /* Kick lai IT nhan - PHAI co, neu khong sensor se im lang mai */
    HAL_UART_Receive_IT(huart, &s_rx_byte[id], 1);

    /* Neu g_dbg_uart_err tang nhanh:
     * - Kiem tra wiring (TX<->RX, GND chung)
     * - Kiem tra baudrate (VB22A mac dinh 460800)
     * - Kiem tra nguon 3.3V cua VB22A */
}