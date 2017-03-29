#ifndef USART_h
#define USART_h

#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define F_CPU 16000000UL
#define UART_BAUD  9600

void usart_init(void);
void usart_transmit(char c);
char usart_receive(void);

#endif
