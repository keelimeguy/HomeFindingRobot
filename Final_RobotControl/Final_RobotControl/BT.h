#ifndef BT_h
#define BT_h

#define F_CPU 16000000UL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "motor.h"
#include "timer.h"

#define PKT_JOYSTICK_X   0
#define PKT_JOYSTICK_Y   1
#define PKT_DISTANCE     2
#define PKT_HEADING      3
#define PKT_DRIFT        4
#define PKT_STRING       5
#define PKT_SET_HOME     6
#define PKT_GO_HOME      7
#define PKT_AUTO_MODE    8
#define PKT_STOP         9
#define PKT_POSITION     10
#define PKT_CONTROL_MODE 11
#define PKT_LINE         12
#define PKT_RIGHT        13
#define PKT_LEFT         14

#define BT_BUFFER_SIZE 15

typedef struct {
    uint8_t pkt_type; // 4 bits used
    uint8_t pkt_len; // 4 bits used
    uint8_t* pkt_val;
} BT_packet_t;

void BT_init(void);
void BT_TimeoutTask(void);
void BT_prepare_for_pkt(void);
void BT_send_pkt(BT_packet_t packet);
BT_packet_t BT_get_pkt();
void BT_send(uint8_t type, uint8_t length, uint8_t* data);
uint8_t BT_has_pkt(void);
uint8_t BT_is_ready(void);

#endif
