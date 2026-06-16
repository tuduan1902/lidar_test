/**
 * lidar_with_encoder.c
 * Tich hop MT6825 encoder + VB22A LiDAR + motor quet
 */

#include "lidar_with_encoder.h"
#include <string.h>
/* ---- Extern handles ---- */

extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart1;  /* -> Jetson  115200 */
extern UART_HandleTypeDef huart2;  /* <->VB22A   460800 */
extern TIM_HandleTypeDef  htim2;   /* PWM motor  CH1=PA0 CH2=PA1 */

/* ---- LED ---- */

#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

/* ============================================================
 *  MT6825 — copy nguyen tu file encoder cua ban
 * ============================================================ */

static uint16_t MT6825_ReadReg(uint8_t reg) {
    uint8_t tx[2] = {0x80 | (reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    HAL_GPIO_WritePin(MT6825_CS_PORT, MT6825_CS_PIN, GPIO_PIN_RESET);
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
/* Lay goc hien tai (degree) tu position tuyet doi */

static float position_to_deg(int64_t pos) {
    /* Lay phan du trong 1 vong */
    int32_t in_rev = (int32_t)(pos % (int64_t)MT6825_RES);
    if (in_rev < 0) in_rev += (int32_t)MT6825_RES;
    float deg = (in_rev * 360.0f) / (float)MT6825_RES;
    /* Chuyen sang -180..+180 */
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

/* ============================================================
 *  Motor control dung position tuyet doi
 *  -> Tranh vong lap vo tan
 * ============================================================ */
static void motor_forward(uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

static void motor_reverse(uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}

static void motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

/* Di chuyen toi target_position (raw units tuyet doi) */

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

/* ============================================================
 *  Trang thai quet: dung position tuyet doi
 *  Bat dau tu vi tri hien tai lam goc (home)
 *  Quet: home - SCAN_RAW_HALF  <->  home + SCAN_RAW_HALF
 * ============================================================ */

static int64_t scan_home      = 0;   /* vi tri luc khoi dong */
static int64_t scan_target    = 0;
static int8_t  scan_dir       = +1;
static uint8_t scan_homed     = 0;

/* ============================================================
 *  VB22A doc khoang cach
 * ============================================================ */
static uint16_t vb22a_read(void) {
    uint8_t f[VB22A_FRAME_LEN] = {0};
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    if (HAL_UART_Receive(&huart2, f, VB22A_FRAME_LEN, 30) != HAL_OK)
        return 0xFFFF;
    if (f[0] != VB22A_HEADER) return 0xFFFF;
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
    return mm;
}

/* ============================================================
 *  Dong goi packet 14 bytes
 * ============================================================ */
static void send_packet(uint16_t dist_mm, float angle_deg) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts    = HAL_GetTick();
    int16_t  ang10 = (int16_t)(angle_deg * 10.0f);
    /* Gui 4 byte thap cua position de debug */
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

    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10);
}

/* ============================================================
 *  INIT
 * ============================================================ */
void lidar_encoder_init(void) {
    LED_OFF();
    HAL_Delay(300);

    /* Khoi dong VB22A */
    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart2, cmd_start, 5, 100);
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
 *  TICK — goi trong while(1)
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
    /* Giu trong khoang -180..+180 */
    while (angle_deg >  180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;

    /* 4. Dieu khien motor toi scan_target */
    motor_goto_position(scan_target);

    /* 5. Kiem tra da toi target chua -> doi chieu */
    int64_t error = scan_target - enc_position;
    if (scan_dir > 0 && error < DEADBAND_RAW) {
        /* Da toi +90, quay ve -90 */
        scan_dir    = -1;
        scan_target = scan_home - SCAN_RAW_HALF;
    } else if (scan_dir < 0 && error > -DEADBAND_RAW) {
        /* Da toi -90, quay ve +90 */
        scan_dir    = +1;
        scan_target = scan_home + SCAN_RAW_HALF;
    }

    /* 6. Doc khoang cach VB22A */
    uint16_t dist = vb22a_read();

    /* 7. Gui packet */
    send_packet(dist, angle_deg);

    /* 8. LED */
    if (dist != 0xFFFF) LED_ON();
    else                LED_OFF();
}

/*

 * THEM VAO main.c:
 *
 *   #include "lidar_with_encoder.h"
 *
 *   // USER CODE BEGIN 2
 *   HAL_TIM_PWM_Start(&tim2, TIM_CHANNEL_1);
 *   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
 *   lidar_encoder_init();
 *   // USER CODE END 2
 *
 *   // USER CODE BEGIN WHILE
 *   while (1) {
 *       lidar_encoder_tick();
 *   // USER CODE END WHILE

 *   }
 *
 * CubeMX / Keil config:
 *   SPI1:  Mode3 (CPOL=High CPHA=2Edge) Master 8bit MSB PSC=32
 *          PA5=SCK PA6=MISO PA7=MOSI PA4=GPIO_Output(CS)
 *   USART1: 115200 8N1  PA9=TX PA10=RX  -> Jetson
 *   USART2: 460800 8N1  PA2=TX PA3=RX   <-> VB22A
 *   TIM2:  PWM CH1+CH2  PSC=71 ARR=999
 *          PA0=CH1(Forward) PA1=CH2(Reverse)
 *   PC13:  GPIO Output (LED)

 */