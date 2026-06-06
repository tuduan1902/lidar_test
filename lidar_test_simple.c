/**
 * lidar_test_simple.c
 * Doc VB22A (460800 baud, frame 4 bytes) -> gui packet Jetson (115200 baud)
 */

#include "lidar_test_simple.h"
#include <string.h>

extern UART_HandleTypeDef huart1;  /* -> Jetson  115200 */
extern UART_HandleTypeDef huart2;  /* <->VB22A   460800 */

#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

/* ----------------------------------------------------------
 * Gui lenh toi VB22A
 * ---------------------------------------------------------- */
static void vb22a_send_cmd(uint8_t* cmd, uint8_t len) {
    HAL_UART_Transmit(&huart2, cmd, len, 100);
}

/* ----------------------------------------------------------
 * Doc 1 frame VB22A (4 bytes)
 * Frame: [0x5C | dist_L dist_H | checksum]
 * checksum = ~(dist_L + dist_H) & 0xFF
 * ---------------------------------------------------------- */
static uint16_t vb22a_read(void) {
    uint8_t f[VB22A_FRAME_LEN] = {0};

    __HAL_UART_CLEAR_OREFLAG(&huart2);

    if (HAL_UART_Receive(&huart2, f, VB22A_FRAME_LEN, 30) != HAL_OK)
        return 0xFFFF;

    /* Validate header */
    if (f[0] != VB22A_HEADER) return 0xFFFF;

    /* Validate checksum: ~(sum bytes[1..2]) */
    uint8_t chk = (uint8_t)(~(f[1] + f[2]) & 0xFF);
    if (chk != f[3]) return 0xFFFF;

    /* Lay khoang cach (mm) */
    uint16_t mm = (uint16_t)f[1] | ((uint16_t)f[2] << 8);

    if (mm < VB22A_MIN_MM || mm >= VB22A_MAX_MM) return 0xFFFF;

    return mm;  /* mm */
}

/* ----------------------------------------------------------
 * Dong goi packet 10 bytes gui Jetson
 * ---------------------------------------------------------- */
static void send_packet(uint16_t dist_mm) {
    uint8_t  pkt[PKT_LEN];
    uint32_t ts = HAL_GetTick();

    pkt[0] = PKT_HEADER;
    pkt[1] = 0x00;
    pkt[2] = (uint8_t)( dist_mm        & 0xFF);
    pkt[3] = (uint8_t)((dist_mm >>  8) & 0xFF);
    pkt[4] = (uint8_t)( ts             & 0xFF);
    pkt[5] = (uint8_t)((ts >>  8)      & 0xFF);
    pkt[6] = (uint8_t)((ts >> 16)      & 0xFF);
    pkt[7] = (uint8_t)((ts >> 24)      & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 7; i++) chk ^= pkt[i];
    pkt[8] = chk;
    pkt[9] = PKT_FOOTER;

    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 10);
}

/* ----------------------------------------------------------
 * Init: gui lenh Start Ranging
 * ---------------------------------------------------------- */
void lidar_test_init(void) {
    LED_OFF();
    HAL_Delay(500);  /* Cho VB22A boot */

    /* Start Ranging command */
    uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
    vb22a_send_cmd(cmd_start, 5);
    HAL_Delay(200);

    /* Bao hieu san sang: nhay 3 lan */
    for (int i = 0; i < 3; i++) {
        LED_ON(); HAL_Delay(100);
        LED_OFF(); HAL_Delay(100);
    }
}

/* ----------------------------------------------------------
 * Tick: doc + gui lien tuc
 * ---------------------------------------------------------- */
void lidar_test_tick(void) {
    uint16_t dist = vb22a_read();
    send_packet(dist);

    /* LED: sang khi co distance hop le */
    if (dist != 0xFFFF) LED_ON();
    else                LED_OFF();
}

/*
 * THEM VAO main.c:
 *
 *   #include "lidar_test_simple.h"
 *
 *   // USER CODE BEGIN 2
 *   lidar_test_init();
 *   // USER CODE END 2
 *
 *   // USER CODE BEGIN WHILE
 *   while (1) {
 *       lidar_test_tick();
 *   // USER CODE END WHILE
 *   }
 */