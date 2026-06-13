/**
 * lidar_fixed.c
 * Lấy dữ liệu từ LiDAR VB22A và gửi thẳng lên Jetson
 */

#include "lidar_encoder.h"

/* ---- Extern handles ---- */
extern UART_HandleTypeDef huart1;  /* -> Jetson  115200 */
extern UART_HandleTypeDef huart2;  /* <->VB22A   460800 */

/* ---- LED ---- */
#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

/* ============================================================
 * VB22A đọc khoảng cách
 * ============================================================ */
static uint16_t vb22a_read(void) {
    uint8_t f[VB22A_FRAME_LEN] = {0};
    
    /* Xóa cờ lỗi ORE (Overrun Error) nếu có để tránh kẹt UART */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    
    if (HAL_UART_Receive(&huart2, f, VB22A_FRAME_LEN, 30) != HAL_OK) {
        return 0xFFFF;
    }
    
    /* Kiểm tra Header và Checksum của VB22A */
    if (f[0] != VB22A_HEADER) return 0xFFFF;
    
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;
    
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;
    
    return mm;
}

/* ============================================================
 * Đóng gói packet 10 bytes gửi lên Jetson
 * ============================================================ */
static void send_packet(uint16_t dist_mm) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts = HAL_GetTick();

    pkt[0]  = PKT_HEADER;
    pkt[1]  = (uint8_t)( dist_mm       & 0xFF);
    pkt[2]  = (uint8_t)((dist_mm >> 8) & 0xFF);
    pkt[3]  = (uint8_t)( ts            & 0xFF);
    pkt[4]  = (uint8_t)((ts >>  8)     & 0xFF);
    pkt[5]  = (uint8_t)((ts >> 16)     & 0xFF);
    pkt[6]  = (uint8_t)((ts >> 24)     & 0xFF);
    pkt[7]  = 0x00; /* Byte đệm (Padding) cho đủ cấu trúc */

    /* Tính Checksum (XOR từ byte 1 đến byte 7) */
    uint8_t chk = 0;
    for (int i = 1; i <= 7; i++) {
        chk ^= pkt[i];
    }
    
    pkt[8] = chk;
    pkt[9] = PKT_FOOTER;

    /* Gửi qua USART1 lên Jetson */
    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10);
}

/* ============================================================
 * INIT
 * ============================================================ */
void lidar_fixed_init(void) {
    LED_OFF();
    HAL_Delay(300);

    /* Gửi lệnh đánh thức/khởi động VB22A (nếu cần) */
    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    HAL_UART_Transmit(&huart2, cmd_start, 5, 100);
    HAL_Delay(200);

    /* Nháy LED báo hiệu sẵn sàng */
    for (int i = 0; i < 3; i++) {
        LED_ON(); HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

/* ============================================================
 * TICK — Gọi liên tục trong while(1) ở main.c
 * ============================================================ */
void lidar_fixed_tick(void) {
    /* Đọc khoảng cách từ VB22A */
    uint16_t dist = vb22a_read();

    /* Đóng gói và gửi thẳng lên Jetson */
    send_packet(dist);

    /* Nháy LED debug: Sáng khi có dữ liệu hợp lệ, tắt khi nhiễu/lỗi */
    if (dist != 0xFFFF) LED_ON();
    else                LED_OFF();
}