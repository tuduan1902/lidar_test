/**
 * dual_lidar_encoder.c — DMA version
 *
 * Core idea:
 *   HAL_UART_Receive_DMA() chay 1 lan khi init, DMA tu dong nap vao
 *   circular buffer lien tuc. tick() chi can scan buffer tim frame 0x5C,
 *   khong bao gio block. Ca 2 UART hoat dong hoan toan song song.
 */
#include "dual_lidar_encoder.h"
#include <string.h>

/* ---- Extern handles ---- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef  hdma_usart2_rx;  /* CubeMX tu tao */
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;

/* ---- DMA circular buffers ---- */
static uint8_t dma_buf[NUM_LIDAR][DMA_BUF_LEN];

/* ---- Lookup ---- */
static UART_HandleTypeDef* const uart_h[NUM_LIDAR]  = {&huart2, &huart3};
static DMA_HandleTypeDef*  const dma_h[NUM_LIDAR]   = {&hdma_usart2_rx, &hdma_usart3_rx};
static TIM_HandleTypeDef*  const tim_h[NUM_LIDAR]   = {&htim2, &htim3};

/* ---- LED ---- */
#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_TOG() HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

/* ============================================================
 *  MT6825 — SPI chung, CS theo id
 * ============================================================ */
static void cs_select(uint8_t id) {
    if (id == LIDAR_LEFT) {
        HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_RESET);
    }
}
static void cs_all_high(void) {
    HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_SET);
}

/* Doc 1 register SPI, KHONG quan ly CS (goi khi CS da o muc thap) */
static uint16_t spi_read_reg(uint8_t reg) {
    uint8_t tx[2] = {0x80 | (reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

static uint32_t enc_read_raw(uint8_t id) {
    // tx[0] = 0x83 (Lệnh đọc từ thanh ghi 0x03). 3 byte sau là dummy (0x00) để kích xung clock đọc r03, r04, r05
    uint8_t tx[4] = {0x83, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0, 0, 0, 0};

    // 1. Kích hoạt chọn chip
    cs_select(id);

    // 2. Truyền nhận liền mạch 4 byte (32 xung clock) - cực kỳ nhanh, không block
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10);

    // 3. Giải phóng chân CS ngay lập tức
    cs_all_high();

    /* Giải mã dữ liệu theo cơ chế Burst Read của MT6825:
     * rx[0]: Byte trạng thái hệ thống (Dummy)
     * rx[1]: Dữ liệu đọc từ thanh ghi 0x03 (Angle[17:10])
     * rx[2]: Dữ liệu đọc từ thanh ghi 0x04 (Angle[9:2])
     * rx[3]: Dữ liệu đọc từ thanh ghi 0x05 (Angle[1:0] nằm ở các bit [7:6])
     */
    uint32_t r03 = rx[1];
    uint32_t r04 = rx[2];
    uint32_t r05 = rx[3];

    // Gộp các bit lại thành góc 18-bit chuẩn xác
    return (r03 << 10) 
         | (r04 << 2) 
         | ((r05 >> 6) & 0x03);
}

/* ---- Multi-turn ---- */
static uint32_t enc_old[NUM_LIDAR]  = {0,0};
static int32_t  enc_turns[NUM_LIDAR]= {0,0};
static int64_t  enc_pos[NUM_LIDAR]  = {0,0};
static uint8_t  enc_first[NUM_LIDAR]= {1,1};

static void enc_update(uint8_t id, uint32_t cur) {
    if (enc_first[id]) { enc_old[id]=cur; enc_first[id]=0; return; }
    int32_t d = (int32_t)cur - (int32_t)enc_old[id];
    if      (d < -131072) enc_turns[id]++;
    else if (d >  131072) enc_turns[id]--;
    enc_old[id] = cur;
}

/* ============================================================
 *  Motor
 * ============================================================ */
static void motor_set(uint8_t id, int8_t dir, uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (dir > 0) {
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_2, 0);
    } else if (dir < 0) {
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_2, pwm);
    } else {
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(tim_h[id], TIM_CHANNEL_2, 0);
    }
}

static void motor_goto(uint8_t id, int64_t target) {
    int64_t err = target - enc_pos[id];
    if (err > DEADBAND_RAW) {
        uint16_t p = (uint16_t)((float)(err>65536?65536:err)*KP_MOTOR);
        motor_set(id, +1, p);
    } else if (err < -DEADBAND_RAW) {
        uint16_t p = (uint16_t)((float)((-err)>65536?65536:(-err))*KP_MOTOR);
        motor_set(id, -1, p);
    } else {
        motor_set(id, 0, 0);
    }
}

/* ---- Scan state ---- */
static int64_t scan_home  [NUM_LIDAR] = {0,0};
static int64_t scan_target[NUM_LIDAR] = {0,0};
static int8_t  scan_dir   [NUM_LIDAR] = {-1,-1};

/* ============================================================
 *  VB22A — doc tu DMA circular buffer (KHONG block)
 *
 *  DMA nap byte lien tuc vao dma_buf[id][0..DMA_BUF_LEN-1]
 *  NDTR (number of data to receive) dem nguoc tu DMA_BUF_LEN ve 0
 *  -> write_pos = DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(dma)
 *
 *  Tim frame: quet buffer tu last_read_pos[id] tim byte 0x5C,
 *  doc 4 byte lien tiep, validate checksum.
 * ============================================================ */
static uint8_t dma_read_pos[NUM_LIDAR] = {0, 0}; /* vi tri da doc */

static uint16_t vb22a_read_dma(uint8_t id) {
    /* Vi tri DMA dang ghi hien tai */
    uint16_t write_pos = DMA_BUF_LEN
                       - (uint16_t)__HAL_DMA_GET_COUNTER(dma_h[id]);

    uint8_t* buf  = dma_buf[id];
    uint8_t  rp   = dma_read_pos[id];

    /* Quet tu rp den write_pos (xu ly wrap-around) */
    uint8_t bytes_avail;
    if (write_pos >= rp)
        bytes_avail = (uint8_t)(write_pos - rp);
    else
        bytes_avail = (uint8_t)(DMA_BUF_LEN - rp + write_pos);

    if (bytes_avail < VB22A_FRAME_LEN) return 0xFFFF; /* chua du frame */

    /* Tim header 0x5C trong bytes co san */
    for (uint8_t i = 0; i < bytes_avail; i++) {
        uint8_t idx = (rp + i) % DMA_BUF_LEN;
        if (buf[idx] != VB22A_HEADER) continue;

        /* Kiem tra du 4 bytes */
        if (bytes_avail - i < VB22A_FRAME_LEN) break;

        /* Doc 4 bytes frame */
        uint8_t f[VB22A_FRAME_LEN];
        for (uint8_t j = 0; j < VB22A_FRAME_LEN; j++)
            f[j] = buf[(idx + j) % DMA_BUF_LEN];

        /* Validate checksum */
        uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
        if (chk != f[3]) {
            /* Checksum sai, bo qua byte nay tim header tiep */
            continue;
        }

        /* Valid frame! Cap nhat read pointer */
        dma_read_pos[id] = (rp + i + VB22A_FRAME_LEN) % DMA_BUF_LEN;

        uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
        if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
        return mm;
    }

    /* Khong tim thay frame hop le, cap nhat read pointer tranh ket */
    dma_read_pos[id] = (uint8_t)write_pos;
    return 0xFFFF;
}

/* ============================================================
 *  Dong goi va gui packet
 * ============================================================ */
static void send_packet(uint8_t id, uint16_t dist_mm, float angle_deg) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts    = HAL_GetTick();
    int16_t  ang10 = (int16_t)(angle_deg * 10.0f);
    uint16_t pos16 = (uint16_t)(enc_pos[id] & 0xFFFF);

    pkt[0]  = PKT_HEADER;
    pkt[1]  = id;
    pkt[2]  = (uint8_t)( dist_mm & 0xFF);
    pkt[3]  = (uint8_t)((dist_mm >> 8) & 0xFF);
    pkt[4]  = (uint8_t)( ang10 & 0xFF);
    pkt[5]  = (uint8_t)((ang10 >> 8) & 0xFF);
    pkt[6]  = (uint8_t)( ts & 0xFF);
    pkt[7]  = (uint8_t)((ts >>  8) & 0xFF);
    pkt[8]  = (uint8_t)((ts >> 16) & 0xFF);
    pkt[9]  = (uint8_t)((ts >> 24) & 0xFF);
    pkt[10] = (uint8_t)( pos16 & 0xFF);
    pkt[11] = (uint8_t)((pos16 >> 8) & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 11; i++) chk ^= pkt[i];
    pkt[12] = chk;
    pkt[13] = PKT_FOOTER;

    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10);
}

/* ============================================================
 *  INIT
 * ============================================================ */
void dual_lidar_init(void) {
    LED_OFF();
    cs_all_high();
    HAL_Delay(300);

    /* Gui Start Ranging cho ca 2 VB22A */
    uint8_t cmd[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart2, cmd, 5, 100);
    HAL_UART_Transmit(&huart3, cmd, 5, 100);
    HAL_Delay(200);

    /* Bat DMA circular cho ca 2 UART truoc tien */
    memset(dma_buf, 0, sizeof(dma_buf));
    HAL_UART_Receive_DMA(&huart2, dma_buf[LIDAR_LEFT],  DMA_BUF_LEN);
    HAL_UART_Receive_DMA(&huart3, dma_buf[LIDAR_RIGHT], DMA_BUF_LEN);


    // /* USER CODE: DEBUG - doc thu DMA buffer cua id=1 sau 500ms */
    // HAL_Delay(500);
    // uint32_t ndtr1 = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    // uint8_t dbg[3] = {0xCC, (uint8_t)(ndtr1 >> 8), (uint8_t)(ndtr1 & 0xFF)};
    // HAL_UART_Transmit(&huart1, dbg, 3, 10);
    // uint8_t debug_pkt[14];
    // uint8_t dbg_hdr = 0xBB;
    // /* Gui 1 byte bao hieu bat dau debug */
    // HAL_UART_Transmit(&huart1, &dbg_hdr, 1, 10);
    // /* Gui toan bo DMA buffer id=1 len Jetson */
    // HAL_UART_Transmit(&huart1, dma_buf[1], DMA_BUF_LEN, 50);

    /* Start PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* HOME encoder */
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        uint32_t raw = enc_read_raw(i);
        enc_update(i, raw);
        enc_pos[i]      = (int64_t)enc_turns[i] * MT6825_RES + raw;
        scan_home[i]    = enc_pos[i];
        scan_target[i]  = scan_home[i] - SCAN_RAW_HALF;
        scan_dir[i]     = -1;
    }

    /* LED nhay 3 lan */
    for (int i = 0; i < 3; i++) {
        LED_ON();  HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

/* ============================================================
 *  TICK — khong block, ca 2 lidar doc song song tu DMA buffer
 * ============================================================ */
void dual_lidar_tick(void) {
    for (uint8_t id = 0; id < NUM_LIDAR; id++) {
        
        /* Chong dong bang DMA do loi Overrun (ORE) */
        UART_HandleTypeDef *huart = uart_h[id];
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
            (huart->RxState == HAL_UART_STATE_READY)) {
            __HAL_UART_CLEAR_OREFLAG(huart);
            HAL_UART_Receive_DMA(huart, dma_buf[id], DMA_BUF_LEN);
        }

        /* 1. Doc encoder */
        uint32_t raw = enc_read_raw(id);

        //Debug: gui raw len Jetson de kiem tra encoder co bi loi khong
        uint8_t dbg[6] = {
            0xDE, id,
            (uint8_t)(raw  &0xFF),
            (uint8_t)((raw >> 8) & 0xFF),
            (uint8_t)((raw >> 16) & 0xFF),

        };
        HAL_UART_Transmit(&huart1, dbg, 6, 5);

        enc_update(id, raw);
        enc_pos[id] = (int64_t)enc_turns[id] * MT6825_RES + raw;

        /* 2. Tinh goc tuong doi */
        int64_t rel = enc_pos[id] - scan_home[id];
        float   deg = (float)rel * 360.0f / (float)MT6825_RES;
        while (deg >  180.0f) deg -= 360.0f;
        while (deg < -180.0f) deg += 360.0f;

        /* 3. Dieu khien motor */
        motor_goto(id, scan_target[id]);

        /* 4. Doi chieu quet */
        int64_t err = scan_target[id] - enc_pos[id];
        if (scan_dir[id] > 0 && err < DEADBAND_RAW) {
            scan_dir[id] = -1;
            scan_target[id] = scan_home[id] - SCAN_RAW_HALF;
        } else if (scan_dir[id] < 0 && err > -DEADBAND_RAW) {
            scan_dir[id] = +1;
            scan_target[id] = scan_home[id] + SCAN_RAW_HALF;
        }

        /* 5. Doc LiDAR tu DMA buffer (non-blocking) */
        uint16_t dist = vb22a_read_dma(id);

        /* 6. Gui packet */
        send_packet(id, dist, deg);

        if (dist != 0xFFFF) LED_TOG();
    }
}

/*
 * ============================================================
 * THEM VAO main.c:
 * ============================================================
 *   #include "dual_lidar_encoder.h"
 *
 *   // USER CODE BEGIN 2
 *   dual_lidar_init();
 *   // USER CODE END 2
 *
 *   // USER CODE BEGIN WHILE
 *   while (1) { dual_lidar_tick(); }
 *   // USER CODE END WHILE
 *
 * ============================================================
 * CUBEMX — THEM MOI so voi blocking version:
 * ============================================================
 *   USART2: them DMA Request RX
 *     -> DMA Settings: DMA1 Channel6, Direction=P->M,
 *        Mode=Circular, DataWidth=Byte/Byte
 *
 *   USART3: them DMA Request RX
 *     -> DMA Settings: DMA1 Channel3, Direction=P->M,
 *        Mode=Circular, DataWidth=Byte/Byte
 *
 *   Sau khi them DMA, CubeMX tu tao:
 *     DMA_HandleTypeDef hdma_usart2_rx;
 *     DMA_HandleTypeDef hdma_usart3_rx;
 *   va cac ham HAL_UART_RxCpltCallback neu can
 *
 *   QUAN TRONG: Trong NVIC, enable:
 *     DMA1 Channel6 global interrupt (cho USART2 RX DMA)
 *     DMA1 Channel3 global interrupt (cho USART3 RX DMA)
 * ============================================================
 */