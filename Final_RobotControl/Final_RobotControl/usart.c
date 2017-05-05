#include "usart.h"

// Initialize the USART library
void usart_init(void) {
    // Set the BAUD rate of the USART
    UBRR0L = (F_CPU / (16UL * UART_BAUD)) - 1;
    UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); /* tx/rx enable */
    UCSR0C = (1<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01); /* tx/rx enable */
}

// Transmit a character over USART
void usart_transmit(char c){
    while(!(UCSR0A & (1<<UDRE0))); // Blocking
    UDR0 = c;
}

// Receive a character over USART
char usart_receive(void){
    while(!(UCSR0A & (1<<RXC0))); // Blocking
    return UDR0;
}
