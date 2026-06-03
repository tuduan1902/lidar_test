/**
 * lidar_test_simple.c
 * Doc VB22A qua USART2, gui packet qua USART1 len Pi.
 * Khong motor, khong encoder, khong angle.
 */

#include "lidar_test_simple.h"

/* CubeIDE/Keil tao trong main.c */
extern UART_HandleTypeDef huart1;  /* -> Pi  921600 */
extern UART_HandleTypeDef huart2;  /* <- VB22A 115200 */

/* ----------------------------------------------------------
 * Doc 1 frame VB22A
 * VB22A chay continuous 200Hz, khong can gui trigger
 * ---------------------------------------------------------- */
static uint16_t vb22a_read(void)
{
    uint8_t f[VB22A_FRAME_LEN] = {0};

    /* Doi toi da 30ms (= 6 frame @ 200Hz) */
    if (HAL_UART_Receive(&huart2, f, VB22A_FRAME_LEN, 30) != HAL_OK)
        return 0xFFFF;

    if (f[0] != VB22A_HEADER || f[1] != VB22A_HEADER)
        return 0xFFFF;

    /* Checksum = sum bytes 0..7, lay byte thap */
    uint8_t chk = 0;
    for (int i = 0; i < 8; i++) chk += f[i];
    if (chk != f[8]) return 0xFFFF;

    uint16_t cm = (uint16_t)f[2] | ((uint16_t)f[3] << 8);
    if (cm == 0 || cm > VB22A_MAX_CM) return 0xFFFF;

    return cm * 10;  /* cm -> mm */
}

/* ----------------------------------------------------------
 * Dong goi va gui
 * ---------------------------------------------------------- */
static void send_packet(uint16_t dist_mm)
{
    uint8_t  pkt[PKT_LEN];
    uint32_t ts = HAL_GetTick();

    pkt[0] = PKT_HEADER;
    pkt[1] = 0x00;                          /* id */
    pkt[2] = (uint8_t)( dist_mm       & 0xFF);
    pkt[3] = (uint8_t)((dist_mm >> 8) & 0xFF);
    pkt[4] = (uint8_t)( ts            & 0xFF);
    pkt[5] = (uint8_t)((ts >>  8)     & 0xFF);
    pkt[6] = (uint8_t)((ts >> 16)     & 0xFF);
    pkt[7] = (uint8_t)((ts >> 24)     & 0xFF);

    uint8_t chk = 0;
    for (int i = 1; i <= 7; i++) chk ^= pkt[i];
    pkt[8] = chk;
    pkt[9] = PKT_FOOTER;

    /* 10 bytes @ 921600 baud ~ 0.11ms, blocking OK */
    HAL_UART_Transmit(&huart1, pkt, PKT_LEN, 5);
}

/* ----------------------------------------------------------
 * API goi tu main.c
 * ---------------------------------------------------------- */
void lidar_test_init(void)
{
    HAL_Delay(200);  /* cho VB22A boot xong */
}

void lidar_test_tick(void)
{
    uint16_t dist = vb22a_read();
    send_packet(dist);
    /* Khong them delay o day:
     * vb22a_read() tu block 0-30ms doi du 1 frame
     * Toc do gui ~ 200 packet/s = khop voi VB22A output rate */
}

/* ----------------------------------------------------------
 * HUONG DAN THEM VAO main.c CUA KEIL / CUBEMX
 * ----------------------------------------------------------
 *
 *   #include "lidar_test_simple.h"
 *
 *   int main(void) {
 *       HAL_Init();
 *       SystemClock_Config();   // 72MHz
 *       MX_GPIO_Init();
 *       MX_USART1_UART_Init();  // 921600
 *       MX_USART2_UART_Init();  // 115200
 *
 *       lidar_test_init();
 *
 *       while (1) {
 *           lidar_test_tick();
 *       }
 *   }
 */