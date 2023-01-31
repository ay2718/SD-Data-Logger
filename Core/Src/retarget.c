/*
 * retarget.c
 *
 *  Created on: Jan 27, 2023
 *      Author: ayeiser
 */


#include <retarget.h>

UART_HandleTypeDef* gHuart;
void RetargetInit(UART_HandleTypeDef* huart) {
	gHuart = huart;
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stdin, NULL, _IONBF, 0);
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;

  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(gHuart);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//	HAL_UART_Transmit(gHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}
