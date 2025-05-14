#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include "main.h"
#include "cmsis_os.h"


#define RING_BUFFER_SIZE 4096
#define MAX_BUFFER_COUNT 256

#define INDEX 0
#define LENGTH 1
#define VALID 2
#define USED 3
#define OVERFLOW 4
typedef struct
{
    uint8_t buf[RING_BUFFER_SIZE];                // 버퍼 크기
    uint16_t buf_index;
    volatile uint16_t index[MAX_BUFFER_COUNT][5]; // 인덱스 배열 0 : 헤드 인덱스, 1 : 테일 인덱스, 2 : 사용 여부 0-미사용, 1-사용, 3 : 유효 여부 0-유효, 1-유효
    // volatile uint8_t head_index;                  // 추가된 버퍼 총 개수
    // volatile uint8_t tail_index;                  // 처리 완료된 인덱스
    volatile uint8_t ring_index
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