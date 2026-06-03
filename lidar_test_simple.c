/**
 * lidar_test_simple.c
 * Doc VB22A qua USART2 (115200), gui packet qua USART1 (460800) len Pi.
 * Khong motor, khong encoder.
 */

#include "lidar_test_simple.h"

extern UART_HandleTypeDef huart1;  /* -> Pi      460800 baud */
extern UART_HandleTypeDef huart2;  /* <- VB22A   115200 baud */

/* ----------------------------------------------------------
 * Doc 1 frame VB22A (9 bytes, continuous output 200Hz)
 *
 * VB22A la single-point: moi frame chi co 1 gia tri distance.
 * Khong can gui trigger -- module tu phat lien tuc.
 *
 * Chieu dai frame: 9 bytes
 * Checksum: CONG (sum), khong XOR
 * ---------------------------------------------------------- */
static uint16_t vb22a_read(void)
{
    uint8_t f[VB22A_FRAME_LEN] = {0};

    /* Timeout 30ms = ~6 frame-period @ 200Hz, du an toan */
    if (HAL_UART_Receive(&huart2, f, VB22A_FRAME_LEN, 30) != HAL_OK)
        return 0xFFFF;

    /* Validate header (2 byte 0x59) */
    if (f[0] != VB22A_HEADER || f[1] != VB22A_HEADER)
        return 0xFFFF;

    /* Checksum: SUM (khong XOR) cua bytes 0..7, lay byte thap */
    uint8_t chk = 0;
    for (int i = 0; i < 8; i++) chk += f[i];
    if (chk != f[8]) return 0xFFFF;

    /* Distance (cm), little-endian */
    uint16_t cm = (uint16_t)f[2] | ((uint16_t)f[3] << 8);

    /* Loc: ngoai tam do hop le */
    if (cm < VB22A_MIN_CM || cm > VB22A_MAX_CM) return 0xFFFF;

    return cm * 10;  /* cm -> mm */
}

/* ----------------------------------------------------------
 * Dong goi packet 10 bytes va gui len Pi
 * ---------------------------------------------------------- */
static void send_packet(uint16_t dist_mm)
{
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

    /* 10 bytes @ 460800 baud ~ 0.22ms -- blocking OK */
    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 5);
}

/* ----------------------------------------------------------
 * API public
 * ---------------------------------------------------------- */
void lidar_test_init(void)
{
    /* Cho VB22A boot va on dinh (datasheet khuyen nghi >100ms) */
    HAL_Delay(500);
}

void lidar_test_tick(void)
{
    uint16_t dist = vb22a_read();
    /* Gui du loi hay hop le: Pi tu loc 0xFFFF */
    send_packet(dist);
    /* Khong delay them: vb22a_read() da block 0..30ms cho frame */
}

/* ----------------------------------------------------------
 * THEM VAO main.c (KEIL):
 * ----------------------------------------------------------
 *
 *   #include "lidar_test_simple.h"
 *
 *   int main(void) {
 *       HAL_Init();
 *       SystemClock_Config();    // 72MHz HSE
 *       MX_GPIO_Init();
 *       MX_USART1_UART_Init();   // Baud=460800, 8N1
 *       MX_USART2_UART_Init();   // Baud=115200,  8N1
 *
 *       lidar_test_init();
 *
 *       while (1) {
 *           lidar_test_tick();
 *       }
 *   }
 */