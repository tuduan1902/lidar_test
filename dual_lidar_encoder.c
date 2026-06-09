/**
 * dual_lidar_encoder.c
 * Mo rong tu lidar_with_encoder.c (1 lidar/encoder/motor)
 * len 2 lidar/encoder/motor bang cach refactor sang array [NUM_LIDAR]
 *
 * Logic hoan toan giong cu, chi nhan doi:
 *   - enc_old_raw, enc_turns, enc_position, scan_home, scan_target, scan_dir
 *     tat ca thanh array [2]
 *   - MT6825_ReadReg/ReadRawAngle nhan them tham so id de chon CS
 *   - motor_forward/reverse nhan them id de chon TIM
 *   - vb22a_read nhan them id de chon UART
 *   - send_packet nhan them id de dien vao pkt[1]
 */

#include "dual_lidar_encoder.h"
#include <string.h>

/* ---- Extern handles (CubeMX) ---- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;   /* -> Jetson   115200 */
extern UART_HandleTypeDef huart2;   /* <- VB22A LEFT  460800 */
extern UART_HandleTypeDef huart3;   /* <- VB22A RIGHT 460800 */
extern TIM_HandleTypeDef  htim2;    /* Motor LEFT  CH1=PA0 CH2=PA1 */
extern TIM_HandleTypeDef  htim3;    /* Motor RIGHT CH1=PB4 CH2=PB5 (partial remap) */

/* ---- Lookup: UART va TIM theo id ---- */
static UART_HandleTypeDef* const uart_lidar[NUM_LIDAR] = { &huart2, &huart3 };
static TIM_HandleTypeDef*  const tim_motor[NUM_LIDAR]  = { &htim2,  &htim3  };

/* ---- LED ---- */
#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_TOG() HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

/* ============================================================
 *  MT6825 — SPI chung, chon CS theo id
 * ============================================================ */

static void cs_select(uint8_t id) {
    /* Keo xuong CS cua encoder can doc, keo len cai con lai */
    if (id == LIDAR_LEFT) {
        HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_RESET);
    }
}

static void cs_release_all(void) {
    HAL_GPIO_WritePin(CS_LEFT_PORT,  CS_LEFT_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_RIGHT_PORT, CS_RIGHT_PIN, GPIO_PIN_SET);
}

static uint16_t MT6825_ReadReg(uint8_t id, uint8_t reg) {
    uint8_t tx[2] = {0x80 | (reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    cs_select(id);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    cs_release_all();
    return ((uint16_t)rx[0] << 8) | rx[1];
}

static uint32_t MT6825_ReadRawAngle(uint8_t id) {
    uint16_t r03 = MT6825_ReadReg(id, 0x03); HAL_Delay(1);
    uint16_t r04 = MT6825_ReadReg(id, 0x04); HAL_Delay(1);
    uint16_t r05 = MT6825_ReadReg(id, 0x05);
    return ((uint32_t)(r03 & 0xFF) << 10)
         | ((uint32_t)(r04 & 0xFF) <<  2)
         | ((uint32_t)(r05 >>  6) & 0x03);
}

/* ============================================================
 *  Multi-turn state -- array [NUM_LIDAR]
 *  (logic giong het file cu, chi them [id])
 * ============================================================ */
static uint32_t enc_old_raw [NUM_LIDAR] = {0, 0};
static int32_t  enc_turns   [NUM_LIDAR] = {0, 0};
static int64_t  enc_position[NUM_LIDAR] = {0, 0};
static uint8_t  enc_first   [NUM_LIDAR] = {1, 1};

static void MT6825_UpdateTurns(uint8_t id, uint32_t cur) {
    if (enc_first[id]) {
        enc_old_raw[id] = cur;
        enc_first[id]   = 0;
        return;
    }
    int32_t diff = (int32_t)cur - (int32_t)enc_old_raw[id];
    if      (diff < -131072) enc_turns[id]++;
    else if (diff >  131072) enc_turns[id]--;
    enc_old_raw[id] = cur;
}

static int64_t MT6825_GetPosition(uint8_t id, uint32_t cur) {
    return (int64_t)enc_turns[id] * 262144LL + (int64_t)cur;
}

/* ============================================================
 *  Motor -- chon TIM theo id, logic giong cu
 * ============================================================ */
static void motor_forward(uint8_t id, uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_2, 0);
}

static void motor_reverse(uint8_t id, uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_2, pwm);
}

static void motor_stop(uint8_t id) {
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(tim_motor[id], TIM_CHANNEL_2, 0);
}

/* Logic goto position -- copy y het file cu, them tham so id */
static void motor_goto_position(uint8_t id, int64_t target) {
    int64_t error = target - enc_position[id];

    if (error > DEADBAND_RAW) {
        uint16_t pwm = (uint16_t)(
            (float)(error > 65536 ? 65536 : error) * KP_MOTOR);
        motor_forward(id, pwm);
    } else if (error < -DEADBAND_RAW) {
        int64_t abs_err = -error;
        uint16_t pwm = (uint16_t)(
            (float)(abs_err > 65536 ? 65536 : abs_err) * KP_MOTOR);
        motor_reverse(id, pwm);
    } else {
        motor_stop(id);
    }
}

/* ============================================================
 *  Scan state -- array [NUM_LIDAR]
 * ============================================================ */
static int64_t scan_home  [NUM_LIDAR] = {0, 0};
static int64_t scan_target[NUM_LIDAR] = {0, 0};
static int8_t  scan_dir   [NUM_LIDAR] = {-1, -1};

/* ============================================================
 *  VB22A -- chon UART theo id, logic giong cu
 * ============================================================ */
static uint16_t vb22a_read(uint8_t id) {
    uint8_t f[VB22A_FRAME_LEN] = {0};
    __HAL_UART_CLEAR_OREFLAG(uart_lidar[id]);
    if (HAL_UART_Receive(uart_lidar[id], f, VB22A_FRAME_LEN, 30) != HAL_OK)
        return 0xFFFF;
    if (f[0] != VB22A_HEADER) return 0xFFFF;
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
    return mm;
}

/* ============================================================
 *  Dong goi packet -- pkt[1] = id (truoc la luon 0x00)
 * ============================================================ */
static void send_packet(uint8_t id, uint16_t dist_mm, float angle_deg) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts    = HAL_GetTick();
    int16_t  ang10 = (int16_t)(angle_deg * 10.0f);
    uint32_t pos_lo = (uint32_t)(enc_position[id] & 0xFFFFFFFF);

    pkt[0]  = PKT_HEADER;
    pkt[1]  = id;                               /* 0=LEFT 1=RIGHT */
    pkt[2]  = (uint8_t)( dist_mm       & 0xFF);
    pkt[3]  = (uint8_t)((dist_mm >> 8) & 0xFF);
    pkt[4]  = (uint8_t)( ang10         & 0xFF);
    pkt[5]  = (uint8_t)((ang10  >> 8)  & 0xFF);
    pkt[6]  = (uint8_t)( ts            & 0xFF);
    pkt[7]  = (uint8_t)((ts >>  8)     & 0xFF);
    pkt[8]  = (uint8_t)((ts >> 16)     & 0xFF);
    pkt[9]  = (uint8_t)((ts >> 24)     & 0xFF);
    pkt[10] = (uint8_t)( pos_lo        & 0xFF);
    pkt[11] = (uint8_t)((pos_lo >> 8)  & 0xFF);

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
    cs_release_all();
    HAL_Delay(300);

    /* Gui lenh Start Ranging cho ca 2 VB22A */
    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart2, cmd_start, 5, 100);  /* LEFT  */
    HAL_UART_Transmit(&huart3, cmd_start, 5, 100);  /* RIGHT */
    HAL_Delay(200);

    /* Khoi dong ca 2 motor PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* Doc HOME cho ca 2 encoder */
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        uint32_t raw = MT6825_ReadRawAngle(i);
        MT6825_UpdateTurns(i, raw);
        enc_position[i] = MT6825_GetPosition(i, raw);
        scan_home[i]    = enc_position[i];
        scan_target[i]  = scan_home[i] - SCAN_RAW_HALF; /* bat dau ve -90 */
        scan_dir[i]     = -1;
    }

    /* LED nhay 3 lan bao hieu san sang */
    for (int i = 0; i < 3; i++) {
        LED_ON();  HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

/* ============================================================
 *  TICK -- goi trong while(1)
 *  Xu ly tuan tu LEFT (id=0) roi RIGHT (id=1)
 *  Logic moi id giong het file cu, chi them [id]
 * ============================================================ */
void dual_lidar_tick(void) {

    for (uint8_t id = 0; id < NUM_LIDAR; id++) {

        /* 1. Doc encoder, cap nhat position */
        uint32_t raw = MT6825_ReadRawAngle(id);
        MT6825_UpdateTurns(id, raw);
        enc_position[id] = MT6825_GetPosition(id, raw);

        /* 2. Tinh goc tuong doi so voi HOME */
        int64_t rel_raw    = enc_position[id] - scan_home[id];
        float   angle_deg  = (float)rel_raw * 360.0f / (float)MT6825_RES;
        while (angle_deg >  180.0f) angle_deg -= 360.0f;
        while (angle_deg < -180.0f) angle_deg += 360.0f;

        /* 3. Dieu khien motor toi scan_target */
        motor_goto_position(id, scan_target[id]);

        /* 4. Kiem tra da toi target -> doi chieu (logic y het file cu) */
        int64_t error = scan_target[id] - enc_position[id];
        if (scan_dir[id] > 0 && error < DEADBAND_RAW) {
            scan_dir[id]    = -1;
            scan_target[id] = scan_home[id] - SCAN_RAW_HALF;
        } else if (scan_dir[id] < 0 && error > -DEADBAND_RAW) {
            scan_dir[id]    = +1;
            scan_target[id] = scan_home[id] + SCAN_RAW_HALF;
        }

        /* 5. Doc khoang cach VB22A */
        uint16_t dist = vb22a_read(id);

        /* 6. Gui packet voi id tuong ung */
        send_packet(id, dist, angle_deg);

        /* 7. LED toggle khi nhan duoc du lieu hop le */
        if (dist != 0xFFFF) LED_TOG();
    }
}

/*
 * ============================================================
 * THEM VAO main.c (THAY THE lidar_encoder_init/tick cu):
 * ============================================================
 *
 *   #include "dual_lidar_encoder.h"
 *
 *   // USER CODE BEGIN 2
 *   dual_lidar_init();
 *   // USER CODE END 2
 *
 *   // USER CODE BEGIN WHILE
 *   while (1) {
 *       dual_lidar_tick();
 *   // USER CODE END WHILE
 *   }
 *
 * ============================================================
 * CUBEMX THEM MOI so voi file cu:
 * ============================================================
 *   1. USART3: 460800 8N1
 *      PB10=TX  PB11=RX
 *
 *   2. GPIO PB0: Output Push-Pull, Speed=Low
 *      (CS encoder RIGHT)
 *
 *   3. TIM3 Partial Remap:
 *      - Mode: PWM Generation CH1 + CH2
 *      - Prescaler=71  Period(ARR)=999
 *      - Partial remap: enable trong CubeMX
 *        -> CH1 doi tu PA6 sang PB4
 *        -> CH2 doi tu PA7 sang PB5
 *      Ly do remap: PA6/PA7 trung voi SPI1 MISO/MOSI
 *
 *   Trong code HAL CubeMX se tu them:
 *      __HAL_AFIO_REMAP_TIM3_PARTIAL();
 *   vao ham HAL_TIM_MspPostInit() trong stm32f1xx_hal_msp.c
 *
 * ============================================================
 * KHONG THAY DOI:
 * ============================================================
 *   SPI1  PA5/PA6/PA7 (SCK/MISO/MOSI)
 *   PA4   CS encoder LEFT
 *   USART1 PA9/PA10   -> Jetson
 *   USART2 PA2/PA3    <- VB22A LEFT
 *   TIM2  PA0/PA1     motor LEFT
 *   PC13  LED
 */