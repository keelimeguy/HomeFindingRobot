#ifndef BT_h
#define BT_h

#define F_CPU 16000000UL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "home_finding_robot.h"

#define PKT_JOYSTICK_X   0
#define PKT_JOYSTICK_Y   1
#define PKT_DISTANCE     2
#define PKT_STRING       3

#define BT_BUFFER_SIZE 15

typedef struct {
    uint8_t pkt_type; // 4 bits used
    uint8_t pkt_len; // 4 bits used
    uint8_t* pkt_val;
} BT_packet_t;

void BT_init(void);
void BT_prepare_for_pkt(void);
void BT_send_pkt(BT_packet_t packet);
BT_packet_t BT_get_pkt();
void BT_send(uint8_t type, uint8_t length, uint8_t* data);
uint8_t BT_has_pkt(void);
uint8_t BT_is_ready(void);

#endif
