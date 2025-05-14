#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include "main.h"
#include "cmsis_os.h"

// 버퍼 크기 정의 - 2의 지수승으로 설정하여 모듈로 연산 최적화
#define RING_BUFFER_SIZE 1024
#define MAX_BUFFER_COUNT 256

// 인덱스 배열의 각 요소 의미 정의
#define INDEX     0  // 버퍼 시작 인덱스
#define LENGTH    1  // 데이터 길이
#define VALID     2  // 유효성 플래그 (0-무효, 1-유효)
#define USED      3  // 사용 여부 플래그 (0-미사용, 1-사용)
#define OVERFLOW  4  // 오버플로우 플래그

// 링 버퍼 구조체 정의
typedef struct
{
    uint8_t buf[RING_BUFFER_SIZE];                // 데이터 버퍼
    uint16_t buf_index;                           // 현재 버퍼 인덱스
    volatile uint16_t index[MAX_BUFFER_COUNT][5]; // 인덱스 배열
    volatile uint16_t ring_index;                 // 현재 링 인덱스
    volatile uint16_t take_index;                 // 가져올 인덱스
} RING_BUFFER;

// 프로토콜 구조체 정의
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

// 함수 선언
void Ring_buffer_init(RING_BUFFER *rb);
int Ring_buffer_put(RING_BUFFER *rb, uint8_t data);
int Ring_buffer_next(RING_BUFFER *rb);
void Ring_buffer_flush(RING_BUFFER *rb);
int Ring_buffer_get_packet(RING_BUFFER *rb, uint8_t *buf, uint16_t *length);

#endif /* _RINGBUFFER_H */