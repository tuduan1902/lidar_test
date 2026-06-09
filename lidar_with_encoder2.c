/**
 * lidar_with_encoder2.c
 * Dual MT6825 encoder + dual VB22A LiDAR + dual motor driver
 * Separate file from lidar_with_encoder.c
 */
#include "lidar_with_encoder2.h"
#include <string.h>

/* ---- Extern handles already declared in header ---- */

/* ---- LED ---- */
#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

static const GPIO_TypeDef* mt_cs_port[MT6825_COUNT] = {
    MT6825_CS_PORT0,
    MT6825_CS_PORT1,
};
static const uint16_t mt_cs_pin[MT6825_COUNT] = {
    MT6825_CS_PIN0,
    MT6825_CS_PIN1,
};

static UART_HandleTypeDef* const vb_uart[VB22A_COUNT] = {
    &huart2,
    &huart3,
};

static TIM_HandleTypeDef* const motor_timer[MOTOR_COUNT] = {
    &htim2,
    &htim3,
};

static uint16_t MT6825_ReadReg(uint8_t id, uint8_t reg) {
    uint8_t tx[2] = {0x80 | (reg & 0x7F), 0x00};
    uint8_t rx[2] = {0, 0};
    HAL_GPIO_WritePin((GPIO_TypeDef*)mt_cs_port[id], mt_cs_pin[id], GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    HAL_GPIO_WritePin((GPIO_TypeDef*)mt_cs_port[id], mt_cs_pin[id], GPIO_PIN_SET);
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

static uint32_t enc_old_raw[MT6825_COUNT] = {0, 0};
static int32_t  enc_turns[MT6825_COUNT]   = {0, 0};
static uint8_t  enc_first[MT6825_COUNT]   = {1, 1};
static int64_t  enc_position[MT6825_COUNT] = {0, 0};

static void MT6825_UpdateTurns(uint8_t id, uint32_t cur) {
    if (enc_first[id]) {
        enc_old_raw[id] = cur;
        enc_first[id] = 0;
        return;
    }
    int32_t diff = (int32_t)cur - (int32_t)enc_old_raw[id];
    if (diff < -131072) enc_turns[id]++;
    else if (diff > 131072) enc_turns[id]--;
    enc_old_raw[id] = cur;
}

static int64_t MT6825_GetPosition(uint8_t id, uint32_t cur) {
    return (int64_t)enc_turns[id] * MT6825_RES + (int64_t)cur;
}

static float position_to_deg(int64_t pos) {
    int32_t in_rev = (int32_t)(pos % (int64_t)MT6825_RES);
    if (in_rev < 0) in_rev += (int32_t)MT6825_RES;
    float deg = (in_rev * 360.0f) / (float)MT6825_RES;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

static void motor_forward(uint8_t id, uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (id == 0) {
        __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_2, 0);
    }
}

static void motor_reverse(uint8_t id, uint16_t pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_2, pwm);
}

static void motor_stop(uint8_t id) {
    __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(motor_timer[id], TIM_CHANNEL_2, 0);
}

static void motor_goto_position(uint8_t id, int64_t target) {
    int64_t error = target - enc_position[id];
    if (error > DEADBAND_RAW) {
        uint16_t pwm = (uint16_t)((float)(error > 65536 ? 65536 : error) * KP_MOTOR);
        motor_forward(id, pwm);
    } else if (error < -DEADBAND_RAW) {
        int64_t abs_err = -error;
        uint16_t pwm = (uint16_t)((float)(abs_err > 65536 ? 65536 : abs_err) * KP_MOTOR);
        motor_reverse(id, pwm);
    } else {
        motor_stop(id);
    }
}

static int64_t scan_home[MT6825_COUNT];
static int64_t scan_target[MT6825_COUNT];
static int8_t  scan_dir[MT6825_COUNT];
static uint8_t scan_homed[MT6825_COUNT];

static uint16_t vb22a_read(uint8_t id) {
    uint8_t f[VB22A_FRAME_LEN] = {0};
    UART_HandleTypeDef* huart = vb_uart[id];
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive(huart, f, VB22A_FRAME_LEN, 30) != HAL_OK) return 0xFFFF;
    if (f[0] != VB22A_HEADER) return 0xFFFF;
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
    return mm;
}

static void send_packet(uint8_t id, uint16_t dist_mm, float angle_deg) {
    uint8_t pkt[PKT_LEN];
    uint32_t ts = HAL_GetTick();
    int16_t ang10 = (int16_t)(angle_deg * 10.0f);
    uint32_t pos_lo = (uint32_t)(enc_position[id] & 0xFFFFFFFF);

    pkt[0] = PKT_HEADER;
    pkt[1] = id;
    pkt[2] = (uint8_t)( dist_mm        & 0xFF);
    pkt[3] = (uint8_t)((dist_mm >> 8)  & 0xFF);
    pkt[4] = (uint8_t)( ang10          & 0xFF);
    pkt[5] = (uint8_t)((ang10 >> 8)   & 0xFF);
    pkt[6] = (uint8_t)( ts             & 0xFF);
    pkt[7] = (uint8_t)((ts >> 8)      & 0xFF);
    pkt[8] = (uint8_t)((ts >> 16)     & 0xFF);
    pkt[9] = (uint8_t)((ts >> 24)     & 0xFF);
    pkt[10] = (uint8_t)( pos_lo        & 0xFF);
    pkt[11] = (uint8_t)((pos_lo >> 8) & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 11; i++) chk ^= pkt[i];
    pkt[12] = chk;
    pkt[13] = PKT_FOOTER;

    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10);
}

void lidar_encoder2_init(void) {
    LED_OFF();
    HAL_Delay(300);

    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart2, cmd_start, 5, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart3, cmd_start, 5, 100);
    HAL_Delay(200);

    for (uint8_t id = 0; id < MT6825_COUNT; id++) {
        uint32_t raw = MT6825_ReadRawAngle(id);
        MT6825_UpdateTurns(id, raw);
        enc_position[id] = MT6825_GetPosition(id, raw);
        scan_home[id] = enc_position[id];
        scan_target[id] = (id == 0)
            ? scan_home[id] - SCAN_RAW_HALF
            : scan_home[id] + SCAN_RAW_HALF;
        scan_dir[id] = (id == 0) ? -1 : +1;
        scan_homed[id] = 1;
    }

    for (int i = 0; i < 3; i++) {
        LED_ON(); HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

void lidar_encoder2_tick(void) {
    for (uint8_t id = 0; id < VB22A_COUNT; id++) {
        uint32_t raw = MT6825_ReadRawAngle(id);
        MT6825_UpdateTurns(id, raw);
        enc_position[id] = MT6825_GetPosition(id, raw);

        float angle_deg = position_to_deg(enc_position[id] - scan_home[id]);
        motor_goto_position(id, scan_target[id]);

        int64_t error = scan_target[id] - enc_position[id];
        if (scan_dir[id] > 0 && error < DEADBAND_RAW) {
            scan_dir[id] = -1;
            scan_target[id] = scan_home[id] - SCAN_RAW_HALF;
        } else if (scan_dir[id] < 0 && error > -DEADBAND_RAW) {
            scan_dir[id] = +1;
            scan_target[id] = scan_home[id] + SCAN_RAW_HALF;
        }

        uint16_t dist = vb22a_read(id);
        send_packet(id, dist, angle_deg);

        if (dist != 0xFFFF) LED_ON();
        else                LED_OFF();
    }
}
