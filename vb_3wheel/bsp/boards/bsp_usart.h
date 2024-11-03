#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "usart.h"

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void usart_printf(const char *fmt,...);

void uart_dma_printf(UART_HandleTypeDef *huart, char *fmt, ...);
#endif
