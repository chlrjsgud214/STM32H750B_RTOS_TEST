#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include "main.h"
#include "cmsis_os.h"


#define RING_BUFFER_SIZE 8192
#define MAX_BUFFER_COUNT 256

#define HEAD 0
#define TAIL 1

typedef struct
{
    uint8_t buf[RING_BUFFER_SIZE];                // 버퍼 크기
    volatile uint16_t index[MAX_BUFFER_COUNT][2]; // 인덱스 배열 0 : 헤드 인덱스, 1 : 테일 인덱스
    volatile uint8_t head_index;                  // 추가된 버퍼 총 개수
    volatile uint8_t tail_index;                  // 처리 완료된 인덱스
} RING_BUFFER;

typedef struct
{
    uint8_t RX_DIV;
    uint8_t RX_ID;
    uint8_t RX_OP;
    char RX_POSITION[4];
    uint8_t RX_CHKSUM;
} _CTRL_PROTOCOL;

typedef struct
{
    uint8_t RX_DIV;
    uint8_t RX_ID;
    uint8_t RX_OP;
    uint8_t RX_CHKSUM;
} _REQ_PROTOCOL;

void Ring_buffer_init(RING_BUFFER *rb);
int Ring_buffer_full(RING_BUFFER *rb);
int Ring_buffer_empty(RING_BUFFER *rb);
int Ring_buffer_put(RING_BUFFER *rb, uint8_t data);
int Ring_buffer_next(RING_BUFFER *rb);
int Ring_buffer_get(RING_BUFFER *rb, uint8_t *data);
void Ring_buffer_flush(RING_BUFFER *rb);
int Ring_buffer_add_packet(RING_BUFFER *rb, uint16_t start, uint16_t end);
int Ring_buffer_get_packet(RING_BUFFER *rb, uint16_t *start, uint16_t *end);
int Ring_buffer_cut_packet(RING_BUFFER *rb);
uint16_t Ring_buffer_usage(RING_BUFFER *rb);
#endif