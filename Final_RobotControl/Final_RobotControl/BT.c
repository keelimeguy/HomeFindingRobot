#include "BT.h"

// Index into receiving packet data queue
static volatile int BT_index;
// State flag for the type of data we are ready to receive
static volatile int BT_ready; // 0: header, 1: data, 2: none

// The packet we are receiving
static BT_packet_t BT_rcv_pkt;

// A circular buffer of packets we have already received
static BT_packet_t BT_buffer[BT_BUFFER_SIZE];
static int BT_buffer_index;
static int BT_buffer_next;
static int BT_buffer_size;
static int BT_buffer_overload;

static void BT_buffer_add(BT_packet_t pkt);

// Called from timer.c when BT timer runs out out
void BT_TimeoutTask(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable USART receive interrupt
    stop();                // stop the robot (motor.c)
    BT_prepare_for_pkt(); // reset BT state
}

// USART recieve interrupt, called when Bluetooth packet is received
ISR(USART_RX_vect) {
    uint8_t c;
    // Two states of packet receiving, we are either waiting on the next header or we have got header and are waiting for data
    switch(BT_ready) {
        case 0: // Get header
            setBTTimeout_ms(-1); // Disable BT timeout (timer.c)
            c = usart_receive(); // Receive data, blocking (usart.c)
            // First four bits of header gives type, last four gives length of data
            BT_rcv_pkt.pkt_type = c>>4;
            BT_rcv_pkt.pkt_len = c & 0xf;
            // Prepare for data if length is not zero
            if (BT_rcv_pkt.pkt_len!=0) {
                // Allocate memory for data, is STRING type, allocate extra bit for string terminating character
                BT_rcv_pkt.pkt_val = malloc(BT_rcv_pkt.pkt_len * sizeof(uint8_t) + (BT_rcv_pkt.pkt_type == PKT_STRING ? 1 : 0));
                BT_index = 0; // Set index in buffer to 0
                BT_ready = 1; // Switch to receiving data state
                // Start BT timeout, if data not recieved within this time, robot stops
                setBTTimeout_ms(100); // timer.c
                break;
            } else { // Zero length means no data is expected
                BT_rcv_pkt.pkt_val = 0;
                BT_buffer_add(BT_rcv_pkt); // Add packet to queue
                BT_prepare_for_pkt(); // Reset state to wait for next packet
                // Start BT timeout, if data not recieved within this time, robot stops
                setBTTimeout_ms(500); // timer.c
                break;
            }
        case 1: // Get data
            setBTTimeout_ms(-1); // Disable BT timeout (timer.c)
            c = usart_receive(); // Receive data, blocking (usart.c)
            BT_rcv_pkt.pkt_val[BT_index] = c; // Store data byte at next index
            BT_index++; // Increment index in received data buffer
            // Check if we reached the length of packet
            if (BT_index >= BT_rcv_pkt.pkt_len) {
                if (BT_rcv_pkt.pkt_type == PKT_STRING)
                    // For strings, add string terminating character
                    BT_rcv_pkt.pkt_val[BT_rcv_pkt.pkt_len] = 0;
                BT_buffer_add(BT_rcv_pkt); // Add packet to queue
                BT_prepare_for_pkt(); // Reset state to wait for next packet
                // Start BT timeout, if data not recieved within this time, robot stops
                setBTTimeout_ms(500); // timer.c
            } else
                // Start BT timeout, if data not recieved within this time, robot stops
                setBTTimeout_ms(100); // timer.c
            break;
    }
}

// Add a BT packet to the queue
static void BT_buffer_add(BT_packet_t pkt) {
    // Only add if we have not exceeded size limit
    if(BT_buffer_size!=BT_BUFFER_SIZE) {
        if (BT_buffer[BT_buffer_next].pkt_len != 0)
            // Free allocated memory if it was allocated
            free(BT_buffer[BT_buffer_next].pkt_val);
        // Copy packet header info to next packet in buffer
        BT_buffer[BT_buffer_next].pkt_type = pkt.pkt_type;
        BT_buffer[BT_buffer_next].pkt_len = pkt.pkt_len;
        if (pkt.pkt_len != 0) {
            // Copy packet data to next packet in buffer, if data exists
            BT_buffer[BT_buffer_next].pkt_val = malloc(pkt.pkt_len * sizeof(uint8_t) + (pkt.pkt_type == PKT_STRING ? 1 : 0));
            if (BT_buffer[BT_buffer_next].pkt_val != 0) {
                memcpy(BT_buffer[BT_buffer_next].pkt_val, pkt.pkt_val, pkt.pkt_len + (pkt.pkt_type == PKT_STRING ? 1 : 0));
            }
        } else
            // If data does not exist, set to zero
            BT_buffer[BT_buffer_next].pkt_val = 0;
        BT_buffer_size++; // Increment the size of buffer
        // Increment buffer index and loop to zero when exceed size limit
        BT_buffer_next++;
        if(BT_buffer_next>=BT_BUFFER_SIZE) BT_buffer_next = 0;
    } else
        // We have exceeded size limit, set overload flag
        BT_buffer_overload = 1;
}

// Initialize the BT library
void BT_init(void) {
    usart_init(); // Init USART (usart.c)
    // Disable USART receive interrupt
    UCSR0B &= ~(1<<RXCIE0);
    // Reset buffer status
    BT_clear_buffer();
    // Mark not ready to receive packets
    BT_ready = 2;
}

// Reset buffer status
void BT_clear_buffer(void) {
    BT_buffer_index = 0;
    BT_buffer_next = 0;
    BT_buffer_size = 0;
    BT_buffer_overload = 0;
}

// Setup state to receive packets
void BT_prepare_for_pkt(void) {
    // Set state to receiving header byte
    BT_ready = 0;
    // Free allocated memory for data in receiving packet if exists
    if (BT_rcv_pkt.pkt_len!=0)
        free(BT_rcv_pkt.pkt_val);
    // Reset header for receiving packet
    BT_rcv_pkt.pkt_type=0;
    BT_rcv_pkt.pkt_len=0;
    UCSR0B |= (1<<RXCIE0); // Enable USART receive interrupt
}

// Send a packet over bluetooth
void BT_send_pkt(BT_packet_t packet) {
    // Transimt header over USART, blocking (usart.c)
    usart_transmit((packet.pkt_type<<4) | (packet.pkt_len & 0xf));
    // Transimt data bytes over USART, blocking (usart.c)
    for (int i = 0; i < packet.pkt_len; i++) {
        usart_transmit(packet.pkt_val[i]);
    }
}

// Returns value if data is ready, also indicates if buffer overloaded
uint8_t BT_has_pkt(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable USART receive interrupt
    // Set first bit in return value if a packet is in the queue, set second bit if queue is overloaded
    uint8_t val = (BT_buffer_size != 0) | (BT_buffer_overload<<1);
    UCSR0B |= (1<<RXCIE0); // Enable USART receive interrupt
    return val;
}

BT_packet_t BT_get_pkt(void) {
    UCSR0B &= ~(1<<RXCIE0); // Disable USART receive interrupt
    // Prepare an empty packet
    BT_packet_t retBuffer = {0,0,0};
    if(BT_buffer_size!=0) { // If a packet is available
        // Fill return packet with the next queued packet
        retBuffer = BT_buffer[BT_buffer_index];
        BT_buffer_size--; // Decrement queue size
        BT_buffer_overload = 0; // Reset buffer overload
        BT_buffer_index++; // Increment next queue index
        // If index exceeds length of buffer, loop back to zero
        if(BT_buffer_index>=BT_BUFFER_SIZE) BT_buffer_index = 0;
    }
    UCSR0B |= (1<<RXCIE0); // Enable USART receive interrupt
    // Return the packet
    return retBuffer;
}

// Send a deconstructed packet over bluetooth
void BT_send(uint8_t type, uint8_t length, uint8_t* data) {
    // Construct a packet from the parameters and send it
    BT_packet_t packet = {type, length, data};
    BT_send_pkt(packet);
}
