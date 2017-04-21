#include "BT.h"

static volatile int BT_index;
static volatile int BT_ready;

static BT_packet_t BT_rcv_pkt;

static BT_packet_t BT_buffer[BT_BUFFER_SIZE];
static int BT_buffer_index;
static int BT_buffer_next;
static int BT_buffer_size;
static int BT_buffer_overload;

static void BT_buffer_add(BT_packet_t pkt);

void BT_TimeoutTask(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable Rx interrupt
    stop();
    BT_prepare_for_pkt();
}

ISR(USART_RX_vect) {
    uint8_t c;
    switch(BT_ready) {
        case 0: // Get header
            setBTTimeout_ms(-1);
            c = usart_receive();
            BT_rcv_pkt.pkt_type = c>>4;
            BT_rcv_pkt.pkt_len = c & 0xf;
            if (BT_rcv_pkt.pkt_len!=0) {
                BT_rcv_pkt.pkt_val = malloc(BT_rcv_pkt.pkt_len * sizeof(uint8_t) + (BT_rcv_pkt.pkt_type == PKT_STRING ? 1 : 0));
                BT_index = 0;
                BT_ready = 1;
                setBTTimeout_ms(100);
                break;
            } else {
                BT_rcv_pkt.pkt_val = 0;
                BT_buffer_add(BT_rcv_pkt);
                BT_prepare_for_pkt();
                setBTTimeout_ms(500);
                break;
            }
        case 1: // Get data
            setBTTimeout_ms(-1);
            c = usart_receive();
            BT_rcv_pkt.pkt_val[BT_index] = c;
            BT_index++;
            if (BT_index >= BT_rcv_pkt.pkt_len) {
                if (BT_rcv_pkt.pkt_type == PKT_STRING)
                    BT_rcv_pkt.pkt_val[BT_rcv_pkt.pkt_len] = 0;
                BT_buffer_add(BT_rcv_pkt);
                BT_prepare_for_pkt();
                setBTTimeout_ms(500);
            } else
                setBTTimeout_ms(100);
            break;
    }
}

static void BT_buffer_add(BT_packet_t pkt) {
    if(BT_buffer_size!=BT_BUFFER_SIZE) {
        if (BT_buffer[BT_buffer_next].pkt_len != 0)
            free(BT_buffer[BT_buffer_next].pkt_val);
        BT_buffer[BT_buffer_next].pkt_type = pkt.pkt_type;
        BT_buffer[BT_buffer_next].pkt_len = pkt.pkt_len;
        if (pkt.pkt_len != 0) {
            // PORTB|=1<<PORTB5;
            BT_buffer[BT_buffer_next].pkt_val = malloc(pkt.pkt_len * sizeof(uint8_t) + (pkt.pkt_type == PKT_STRING ? 1 : 0));
            if (BT_buffer[BT_buffer_next].pkt_val != 0) {
                // PORTB&=~(1<<PORTB5);
                memcpy(BT_buffer[BT_buffer_next].pkt_val, pkt.pkt_val, pkt.pkt_len + (pkt.pkt_type == PKT_STRING ? 1 : 0));
            }
        } else
            BT_buffer[BT_buffer_next].pkt_val = 0;
        BT_buffer_size++;
        BT_buffer_next++;
        if(BT_buffer_next>=BT_BUFFER_SIZE) BT_buffer_next = 0;
    } else
        BT_buffer_overload = 1;
}

void BT_init(void) {
    usart_init();
    UCSR0B &= ~(1<<RXCIE0);
    BT_buffer_index = 0;
    BT_buffer_next = 0;
    BT_buffer_size = 0;
    BT_buffer_overload = 0;
    BT_ready = 2;
}

void BT_clear_buffer(void) {
    BT_buffer_index = 0;
    BT_buffer_next = 0;
    BT_buffer_size = 0;
    BT_buffer_overload = 0;
}

void BT_prepare_for_pkt(void) {
    BT_ready = 0;
    if (BT_rcv_pkt.pkt_len!=0)
        free(BT_rcv_pkt.pkt_val);
    BT_rcv_pkt.pkt_type=0;
    BT_rcv_pkt.pkt_len=0;
    UCSR0B |= (1<<RXCIE0); // Enable Rx interrupt
}

void BT_send_pkt(BT_packet_t packet) {
    usart_transmit((packet.pkt_type<<4) | (packet.pkt_len & 0xf));
    for (int i = 0; i < packet.pkt_len; i++) {
        usart_transmit(packet.pkt_val[i]);
    }
}

uint8_t BT_has_pkt(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable Rx interrupt
    uint8_t val = (BT_buffer_size != 0) | (BT_buffer_overload<<1);
    BT_buffer_overload = 0;
    UCSR0B |= (1<<RXCIE0); // Enable Rx interrupt
    return val;
}

BT_packet_t BT_get_pkt(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable Rx interrupt
    BT_packet_t retBuffer = {0,0,0};
    if(BT_buffer_size!=0) {
        retBuffer = BT_buffer[BT_buffer_index];
        BT_buffer_size--;
        BT_buffer_index++;
        if(BT_buffer_index>=BT_BUFFER_SIZE) BT_buffer_index = 0;
    }
    UCSR0B |= (1<<RXCIE0); // Enable Rx interrupt
    return retBuffer;
}

void BT_send(uint8_t type, uint8_t length, uint8_t* data) {
    BT_packet_t packet = {type, length, data};
    BT_send_pkt(packet);
}
