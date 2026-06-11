/**
 * lidar_with_encoder.c
 * Tich hop MT6825 encoder + VB22A LiDAR + motor quet
 * Da chuyen doi sang USART3 va TIM3 (Partial Remap)
 */
#include "lidar_with_encoder.h"
#include <string.h>

/* ---- Extern handles ---- */
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;  /* -> Jetson   115200 (PA9/PA10) */
extern UART_HandleTypeDef huart3;  /* <-> VB22A   460800 (PB10/PB11) - THAY DOI */
extern TIM_HandleTypeDef  htim3;   /* PWM motor   CH1=PB4 CH2=PB5    - THAY DOI */

/* ---- LED ---- */
#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

/* ============================================================
 * MT6825 — Giữ nguyên logic đọc qua SPI1
 * ============================================================ */
static uint16_t MT6825_ReadReg(uint8_t reg) {
    uint8_t tx[2] = {0x80 | (reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    HAL_GPIO_WritePin(MT6825_CS_PORT, MT6825_CS_PIN, GPIO_PIN_RESET); /* Cấu hình trong file .h là PB0 */
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    HAL_GPIO_WritePin(MT6825_CS_PORT, MT6825_CS_PIN, GPIO_PIN_SET);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

static uint32_t MT6825_ReadRawAngle(void) {
    uint16_t r03 = MT6825_ReadReg(0x03); HAL_Delay(1);
    uint16_t r04 = MT6825_ReadReg(0x04); HAL_Delay(1);
    uint16_t r05 = MT6825_ReadReg(0x05);
    return ((uint32_t)(r03 & 0xFF) << 10)
         | ((uint32_t)(r04 & 0xFF) <<  2)
         | ((uint32_t)(r05 >>  6) & 0x03);
}

/* ---- Multi-turn state ---- */
static uint32_t enc_old_raw  = 0;
static int32_t  enc_turns    = 0;
static uint8_t  enc_first    = 1;
static int64_t  enc_position = 0;  /* raw units tuyet doi */

static void MT6825_UpdateTurns(uint32_t cur) {
    if (enc_first) { enc_old_raw = cur; enc_first = 0; return; }
    int32_t diff = (int32_t)cur - (int32_t)enc_old_raw;
    if      (diff < -131072) enc_turns++;
    else if (diff >  131072) enc_turns--;
    enc_old_raw = cur;
}

static int64_t MT6825_GetPosition(uint32_t cur) {
    return (int64_t)enc_turns * 262144LL + (int64_t)cur;
}

static float position_to_deg(int64_t pos) {
    int32_t in_rev = (int32_t)(pos % (int64_t)MT6825_RES);
    if (in_rev < 0) in_rev += (int32_t)MT6825_RES;
    float deg = (in_rev * 360.0f) / (float)MT6825_RES;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

/* ============================================================
 * Motor control chuyển sang dùng bộ điều khiển TIM3 (PB4/PB5)
 * ============================================================ */
static void motor_forward(uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm); /* Sửa sang htim3 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
static void motor_reverse(uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);   /* Sửa sang htim3 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}
static void motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);   /* Sửa sang htim3 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

static void motor_goto_position(int64_t target) {
    int64_t error = target - enc_position;

    if (error > DEADBAND_RAW) {
        uint16_t pwm = (uint16_t)((float)(error > 65536 ? 65536 : error) * KP_MOTOR);
        motor_forward(pwm);
    } else if (error < -DEADBAND_RAW) {
        int64_t abs_err = -error;
        uint16_t pwm = (uint16_t)((float)(abs_err > 65536 ? 65536 : abs_err) * KP_MOTOR);
        motor_reverse(pwm);
    } else {
        motor_stop();
    }
}

/* ---- Trang thai quet ---- */
static int64_t scan_home      = 0;
static int64_t scan_target    = 0;
static int8_t  scan_dir       = +1;
static uint8_t scan_homed     = 0;

/* ============================================================
 * VB22A đọc khoảng cách chuyển sang nhận dữ liệu từ USART3
 * ============================================================ */
static uint16_t vb22a_read(void) {
    uint8_t f[VB22A_FRAME_LEN] = {0};
    __HAL_UART_CLEAR_OREFLAG(&huart3); /* Sửa sang huart3 */
    if (HAL_UART_Receive(&huart3, f, VB22A_FRAME_LEN, 30) != HAL_OK) /* Sửa sang huart3 */
        return 0xFFFF;
    if (f[0] != VB22A_HEADER) return 0xFFFF;
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
    return mm;
}

/* ============================================================
 * Dong goi packet 14 bytes gửi lên Jetson qua USART1
 * ============================================================ */
static void send_packet(uint16_t dist_mm, float angle_deg) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts    = HAL_GetTick();
    int16_t  ang10 = (int16_t)(angle_deg * 10.0f);
    uint32_t pos_lo = (uint32_t)(enc_position & 0xFFFFFFFF);

    pkt[0]  = PKT_HEADER;
    pkt[1]  = 0x00;
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

    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10); /* Vẫn giữ huart1 bắn lên Jetson */
}

/* ============================================================
 * INIT
 * ============================================================ */
void lidar_encoder_init(void) {
    LED_OFF();
    HAL_Delay(300);

    /* Khoi dong VB22A qua USART3 */
    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart3, cmd_start, 5, 100); /* Sửa sang huart3 */
    HAL_Delay(200);

    /* Doc vi tri hien tai lam HOME */
    uint32_t raw = MT6825_ReadRawAngle();
    MT6825_UpdateTurns(raw);
    enc_position = MT6825_GetPosition(raw);
    scan_home    = enc_position;

    /* Quet bat dau tu -90 deg */
    scan_target  = scan_home - SCAN_RAW_HALF;
    scan_dir     = -1;
    scan_homed   = 1;

    /* Nhay LED bao hieu san sang */
    for (int i = 0; i < 3; i++) {
        LED_ON(); HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

/* ============================================================
 * TICK — goi trong while(1)
 * ============================================================ */
void lidar_encoder_tick(void) {
    /* 1. Doc encoder, cap nhat position */
    uint32_t raw = MT6825_ReadRawAngle();
    MT6825_UpdateTurns(raw);
    enc_position = MT6825_GetPosition(raw);

    /* 2. Tinh goc hien tai so voi HOME (don vi: raw) */
    int64_t rel_raw = enc_position - scan_home;

    /* 3. Chuyen sang degree (-180..+180) */
    float angle_deg = (float)rel_raw * 360.0f / (float)MT6825_RES;
    while (angle_deg >  180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;

    /* 4. Dieu khien motor toi scan_target */
    motor_goto_position(scan_target);

    /* 5. Kiem tra da toi target chua -> doi chieu */
    int64_t error = scan_target - enc_position;
    if (scan_dir > 0 && error < DEADBAND_RAW) {
        scan_dir    = -1;
        scan_target = scan_home - SCAN_RAW_HALF;
    } else if (scan_dir < 0 && error > -DEADBAND_RAW) {
        scan_dir    = +1;
        scan_target = scan_home + SCAN_RAW_HALF;
    }

    /* 6. Doc khoang cach VB22A tu USART3 */
    uint16_t dist = vb22a_read();

    /* 7. Gui packet lên Jetson */
    send_packet(dist, angle_deg);

    /* 8. LED hiển thị trạng thái nhận tia */
    if (dist != 0xFFFF) LED_ON();
    else                LED_OFF();
}

/*
 * HUONG DAN THEM VAO main.c CHINH XAC:
 *
 * #include "lidar_with_encoder.h"
 *
 * // USER CODE BEGIN 2
 * HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Chuyển sang kích hoạt TIM3
 * HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Chuyển sang kích hoạt TIM3
 * lidar_encoder_init();
 * // USER CODE END 2
 *
 * // USER CODE BEGIN WHILE
 * while (1) {
 * lidar_encoder_tick();
 * // USER CODE END WHILE
 * }
 *
 * Tóm tắt sơ đồ chân thực tế sau cấu hình:
 * SPI1:   PA5=SCK, PA6=MISO, PA7=MOSI
 * PB0:    GPIO_Output (Chân CS_ENC_RIGHT mới của bạn)
 * USART1: PA9=TX, PA10=RX -> Giao tiếp Jetson (115200 8N1)
 * USART3: PB10=TX, PB11=RX <-> Giao tiếp LiDAR VB22A (460800 8N1)
 * TIM3:   PB4=CH1 (Kéo thuận), PB5=CH2 (Kéo nghịch) - [Partial Remap]
 * PC13:   LED báo tín hiệu
 */