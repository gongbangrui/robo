/**
 * stm32f4xx_it.c — interrupt service routines (template sections)
 *
 * Paste each block into the matching location in your CubeMX-generated
 * stm32f4xx_it.c.
 */

/* ============================================================
 * USART1_IRQHandler — gray sensor idle-line detection
 *
 * Paste inside "USER CODE BEGIN USART1_IRQn 1":
 * ============================================================
void USART1_IRQHandler(void)
{
  // ... HAL_UART_IRQHandler(&huart1); ...

  // USER CODE BEGIN USART1_IRQn 1

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);
      gray_sensor_on_rx_idle(&huart1);
  }

  // USER CODE END USART1_IRQn 1
}
 */

/* ============================================================
 * USART1 DMA init — add after MX_USART1_UART_Init() in main.c
 *
 * In main.c USER CODE BEGIN 2, after gray_sensor_init():
 *
 *   // Enable UART idle-line detection
 *   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
 *
 *   // Start DMA reception (buffer defined in gray_sensor.c)
 *   // Note: gray_sensor.c declares rx_buf[32] as its DMA buffer.
 *   // Expose it or use a separate global buffer.
 *   extern uint8_t gray_rx_dma_buf[32];
 *   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gray_rx_dma_buf, sizeof(gray_rx_dma_buf));
 *   __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);  // disable half-transfer interrupt
 *
 * ============================================================ */

/* ============================================================
 * USART6 — debug printf (TX only, no IRQ needed)
 *
 * In main.c USER CODE BEGIN PV:
 *
 *   int __io_putchar(int ch) {
 *       HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 10);
 *       return ch;
 *   }
 *
 * Then use printf() anywhere for debug output.
 * Connect a USB-TTL to PG14 (TX) and GND, 115200 8N1.
 * ============================================================ */
