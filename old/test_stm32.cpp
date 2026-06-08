#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
		#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

		/* Buoc 1: Gui lenh doi baud rate ve 115200
		 * Phai gui o baud 460800 truoc (mac dinh VB22A)
		 * nen can doi USART2 tam thoi len 460800 */
		HAL_UART_DeInit(&huart2);
		huart2.Init.BaudRate = 460800;
		HAL_UART_Init(&huart2);

		uint8_t cmd_baud[] = {0x5A, 0x06, 0x02, 0x80, 0x04, 0x73};
		HAL_UART_Transmit(&huart2, cmd_baud, 6, 100);
		HAL_Delay(200);

		/* Buoc 2: Doi USART2 ve 115200 */
		HAL_UART_DeInit(&huart2);
		huart2.Init.BaudRate = 115200;
		HAL_UART_Init(&huart2);
		HAL_Delay(100);

		/* Buoc 3: Gui lenh Start Ranging */
		uint8_t cmd_start[] = {0x5A, 0x0A, 0x02, 0x02, 0xF1};
		HAL_UART_Transmit(&huart2, cmd_start, 5, 100);
		HAL_Delay(200);

		uint8_t f[4]   = {0};
		uint8_t out[6] = {0};

		while(1) {
				__HAL_UART_CLEAR_OREFLAG(&huart2);

				HAL_StatusTypeDef st = HAL_UART_Receive(&huart2, f, 4, 50);

				out[0] = 0xCC;
				out[1] = (st == HAL_OK) ? 0x01 : 0x00;
				out[2] = f[0]; out[3] = f[1];
				out[4] = f[2]; out[5] = f[3];
				HAL_UART_Transmit(&huart1, out, 6, 10);

				if (st == HAL_OK) LED_ON();
				else              LED_OFF();
		}