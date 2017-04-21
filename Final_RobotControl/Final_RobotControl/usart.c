#include "usart.h"

void usart_init(void) {
    UBRR0L = (F_CPU / (16UL * UART_BAUD)) - 1;
    UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); /* tx/rx enable */
    UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01); /* tx/rx enable */
}

void usart_transmit(char c){
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

char usart_receive(void){
    while(!(UCSR0A & (1<<RXC0)));
    return UDR0;
}
